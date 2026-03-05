"""
UTM System - FastAPI Backend  (v3.1)
4-D Trajectory-Based Conflict Resolution

Request lifecycle:
  1.  Validate inputs (area, no-fly zone, water)
  2.  Plan 4-D trajectory (OSRM + climb/descent + sigma_t)
  3.  Reserve pad slots  (FIFO terminal-area queue)
  4.  Run CPA conflict cascade  (altitude → delay)
  5.  Re-reserve pad slots with final (post-delay) ETAs
  6.  Convert to legacy Trajectory for frontend
  7.  Broadcast mission_created → spawn drone in Cesium
  8.  Background telemetry loop  (2 Hz position + battery)
  9.  Auto-cleanup after landing  (pad release + entity removal)
"""

import asyncio
import math
import random
import time
import uuid
import os
from contextlib import asynccontextmanager
from typing import Any, Dict, List, Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

from models import (
    DeliveryRequest, Mission, Trajectory, Trajectory4D,
    DroneStatus, Position, ConflictAlert, SystemStatus,
)
from conflict_detection import (
    has_any_conflict,
    resolve_conflict,
    trajectories_conflict,
)
import config
import geofencing
import conflict_detection
import pathfinding
import gps_denied
import anchors as anchor_registry


# ══════════════════════════════════════════════════════════════════════════════
# LIFESPAN  (startup / shutdown)
# ══════════════════════════════════════════════════════════════════════════════

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Start background tasks on boot; cancel cleanly on shutdown."""
    telemetry_task = asyncio.create_task(_telemetry_loop())
    print("[UTM] System online — telemetry loop started")
    yield
    telemetry_task.cancel()
    try:
        await telemetry_task
    except asyncio.CancelledError:
        pass
    print("[UTM] System offline")


# ══════════════════════════════════════════════════════════════════════════════
# APP + MIDDLEWARE
# ══════════════════════════════════════════════════════════════════════════════

app = FastAPI(
    title="UTM System API",
    description="Unmanned Traffic Management — 4D Conflict Resolution",
    version="3.1.0",
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=config.CORS_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.mount("/static", StaticFiles(directory="frontend"), name="static")


# ══════════════════════════════════════════════════════════════════════════════
# GLOBAL STATE
# ══════════════════════════════════════════════════════════════════════════════

# mission_id  → Mission        (active / in-flight)
active_missions: Dict[str, Mission] = {}

# drone_id    → Trajectory4D   (approved 4-D plans used for CPA checks)
flight_plans_4d: Dict[str, Trajectory4D] = {}

# ── GPS-Denied per-drone persistent state (module-level) ─────────────────────
# Keyed by drone_id. Survives across telemetry ticks — function-local dicts
# would be fine too (the loop never restarts) but module-level is clearer.
_wls_state:         Dict[str, Any] = {}  # drone_id → last WLSResult
_wls_wp_idx:        Dict[str, int] = {}  # drone_id → last visited 4D waypoint index
_wls_active_nodes:  Dict[str, list] = {} # drone_id → active node_id list (from last WLS fire)

# drone_id    → Trajectory     (legacy, used for telemetry interpolation)
flight_plans_legacy: Dict[str, Trajectory] = {}

stats: Dict[str, int] = {
    "total_missions":     0,
    "conflicts_detected": 0,
    "conflicts_resolved": 0,
}


# ══════════════════════════════════════════════════════════════════════════════
# WEBSOCKET MANAGER
# ══════════════════════════════════════════════════════════════════════════════

class ConnectionManager:
    def __init__(self) -> None:
        self.connections: List[WebSocket] = []

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        self.connections.append(ws)
        print(f"[WS] Client connected  ({len(self.connections)} total)")

    def disconnect(self, ws: WebSocket) -> None:
        if ws in self.connections:
            self.connections.remove(ws)
            print(f"[WS] Client disconnected  ({len(self.connections)} remaining)")

    async def broadcast(self, message: dict) -> None:
        dead: List[WebSocket] = []
        for ws in self.connections:
            try:
                await ws.send_json(message)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.disconnect(ws)

    async def send_to(self, ws: WebSocket, message: dict) -> None:
        try:
            await ws.send_json(message)
        except Exception:
            self.disconnect(ws)


manager = ConnectionManager()


# ══════════════════════════════════════════════════════════════════════════════
# TELEMETRY SIMULATION LOOP
# ══════════════════════════════════════════════════════════════════════════════

def _interpolate_position(traj: Trajectory, t: float) -> Optional[Position]:
    """
    Linear interpolation of position at wall-clock time t.
    Returns None if t is outside the trajectory window.
    """
    wps = traj.waypoints
    if not wps:
        return None
    if t <= wps[0].eta:
        return wps[0].position
    if t >= wps[-1].eta:
        return wps[-1].position
    for i in range(len(wps) - 1):
        w0, w1 = wps[i], wps[i + 1]
        if w0.eta <= t <= w1.eta:
            dt = w1.eta - w0.eta
            if dt < 1e-6:
                return w0.position
            r = (t - w0.eta) / dt
            return Position(
                latitude =w0.position.latitude  + r * (w1.position.latitude  - w0.position.latitude),
                longitude=w0.position.longitude + r * (w1.position.longitude - w0.position.longitude),
                altitude =w0.position.altitude  + r * (w1.position.altitude  - w0.position.altitude),
            )
    return None


def _estimate_battery(traj: Trajectory, t: float) -> float:
    """
    Linearly discharge battery over the flight window.
    Starts at 100 %, ends at (100 - estimated_battery_usage) %.
    Clamps to [0, 100].
    """
    wps = traj.waypoints
    if not wps or traj.total_time <= 0:
        return 100.0
    t_start = wps[0].eta
    t_end   = wps[-1].eta
    elapsed  = max(0.0, min(t - t_start, t_end - t_start))
    fraction = elapsed / (t_end - t_start)
    battery  = 100.0 - fraction * traj.estimated_battery_usage
    return max(0.0, min(100.0, battery))


async def _telemetry_loop() -> None:
    """
    Broadcast position + battery for every active drone at 2 Hz.

    GPS-Denied Navigation — three state dicts (module-level, persistent):
      _wls_state[drone_id]        → last WLSResult (hdop_ok, vdop_ok, etc.)
      _wls_wp_idx[drone_id]       → last 4D waypoint index visited
      _wls_active_nodes[drone_id] → node IDs from the last WLS fire

    These are set once when a WLS fires and PERSIST between waypoints so the
    broadcast always contains meaningful GPS-denied data, not null.
    """
    interval = 1.0 / config.TELEMETRY_UPDATE_RATE   # 0.5 s

    while True:
        try:
            await asyncio.sleep(interval)
            if not active_missions or not manager.connections:
                continue

            now = time.time()

            # ── GC: purge WLS state for drones no longer in active_missions ──
            # _cleanup_mission() removes drones from active_missions, but cannot
            # reach these three module-level dicts. Without this sweep they
            # accumulate one entry per completed drone and never shrink.
            live_ids = {m.drone_id for m in active_missions.values()}
            for ghost in [d for d in list(_wls_state) if d not in live_ids]:
                _wls_state.pop(ghost, None)
                _wls_wp_idx.pop(ghost, None)
                _wls_active_nodes.pop(ghost, None)

            for mission_id, mission in list(active_missions.items()):
                drone_id = mission.drone_id

                traj_legacy = flight_plans_legacy.get(drone_id)
                if not traj_legacy or not traj_legacy.waypoints:
                    continue

                traj_4d = flight_plans_4d.get(drone_id)

                planned_pos = _interpolate_position(traj_legacy, now)
                if planned_pos is None:
                    continue

                # ── WLS checkpoint detection ──────────────────────────────────
                correction_dx = correction_dy = correction_dz = 0.0

                if traj_4d and traj_4d.waypoints:
                    wps_4d = traj_4d.waypoints

                    # Current waypoint = last whose eta has passed
                    current_wp_idx = 0
                    for i, wp in enumerate(wps_4d):
                        if wp.eta <= now:
                            current_wp_idx = i

                    prev_wp_idx = _wls_wp_idx.get(drone_id, -1)

                    if current_wp_idx > prev_wp_idx:
                        # ── New checkpoint reached ────────────────────────────
                        _wls_wp_idx[drone_id] = current_wp_idx
                        wp_now = wps_4d[current_wp_idx]

                        if wp_now.anchor_bindings:
                            planned_xyz = anchor_registry._to_xyz(
                                wp_now.position.latitude,
                                wp_now.position.longitude,
                                wp_now.position.altitude,
                            )
                            # Simulate IMU drift
                            drift_sigma = 5.0
                            true_xyz = (
                                planned_xyz[0] + random.gauss(0.0, drift_sigma),
                                planned_xyz[1] + random.gauss(0.0, drift_sigma),
                                planned_xyz[2] + random.gauss(0.0, drift_sigma * 0.3),
                            )
                            measurements = gps_denied.simulate_ranging(
                                wp_now.anchor_bindings, true_xyz
                            )
                            result = gps_denied.compute_wls_correction(
                                wp_now.anchor_bindings, measurements, planned_xyz,
                            )
                            # ── Persist state ─────────────────────────────────
                            _wls_state[drone_id]        = result
                            _wls_active_nodes[drone_id] = [
                                b.node_id for b in wp_now.anchor_bindings
                            ]
                            print(
                                f"[{mission_id}] WLS @ wp{current_wp_idx}"
                                f"  nodes={result.n_nodes}"
                                f"  HDOP_ok={result.hdop_ok}"
                                f"  VDOP_ok={result.vdop_ok}"
                                f"  Δ=({result.delta_xyz[0]:+.2f},"
                                f"{result.delta_xyz[1]:+.2f},"
                                f"{result.delta_xyz[2]:+.2f})m"
                                f"  RMS={result.residual_rms:.3f}m"
                            )

                    # ── Apply correction within tau window ────────────────────
                    wls_result = _wls_state.get(drone_id)
                    if wls_result is not None:
                        wp_now = wps_4d[current_wp_idx]
                        time_since_wp = now - wp_now.eta
                        if 0.0 <= time_since_wp <= config.WLS_CORRECTION_TAU:
                            t_frac = time_since_wp / config.WLS_CORRECTION_TAU
                            correction_dx = wls_result.delta_xyz[0] * (1.0 - t_frac)
                            correction_dy = wls_result.delta_xyz[1] * (1.0 - t_frac)
                            correction_dz = wls_result.delta_xyz[2] * (1.0 - t_frac)

                # ── Apply geographic correction ───────────────────────────────
                LAT_M = 111_319.5
                LON_M = 111_319.5
                corrected_lat = planned_pos.latitude  + correction_dx / LAT_M
                corrected_lon = planned_pos.longitude + correction_dy / (
                    LON_M * math.cos(math.radians(planned_pos.latitude))
                )
                corrected_alt = max(0.0, min(
                    planned_pos.altitude + correction_dz, 130.0
                ))

                battery = _estimate_battery(traj_legacy, now)

                # ── Read persisted GPS-denied state ───────────────────────────
                # Always use the LAST stored WLS result, not just the current
                # correction window. This ensures hdop_ok / vdop_ok / nodes
                # are always populated in the panel after the first WLS fire.
                wls_result     = _wls_state.get(drone_id)
                active_nodes   = _wls_active_nodes.get(drone_id, [])
                n_anchor_nodes = wls_result.n_nodes if wls_result else 0

                correction_mag = math.sqrt(
                    correction_dx**2 + correction_dy**2 + correction_dz**2
                )

                await manager.broadcast({
                    "type": "telemetry",
                    "data": {
                        "drone_id":      drone_id,
                        "mission_id":    mission_id,
                        "position": {
                            "latitude":  corrected_lat,
                            "longitude": corrected_lon,
                            "altitude":  corrected_alt,
                        },
                        "battery_level": round(battery, 1),
                        "status":        "en_route_delivery",
                        "timestamp":     now,
                        "gps_denied": {
                            # n_nodes from last WLS result — not zero between waypoints
                            "anchor_nodes_used": n_anchor_nodes,
                            "hdop_ok":     wls_result.hdop_ok     if wls_result else None,
                            "vdop_ok":     wls_result.vdop_ok     if wls_result else None,
                            "correction_m": round(correction_mag, 3),
                            "residual_rms": round(wls_result.residual_rms, 3) if wls_result else None,
                            # Persisted from last WLS fire — stays non-empty between waypoints
                            "active_node_ids": active_nodes,
                        },
                    },
                })

        except asyncio.CancelledError:
            raise
        except Exception as e:
            import traceback
            print(f"[Telemetry] Error: {e}")
            traceback.print_exc()


# ══════════════════════════════════════════════════════════════════════════════
# ROUTES — STATIC / INFO
# ══════════════════════════════════════════════════════════════════════════════

@app.get("/")
async def root():
    for path in [
        "frontend/index.html",
        os.path.join(os.path.dirname(__file__), "frontend", "index.html"),
    ]:
        if os.path.exists(path):
            with open(path, encoding="utf-8") as f:
                return HTMLResponse(f.read())
    raise HTTPException(404, "Frontend not found")


@app.get("/api/health")
async def health():
    return {
        "status":          "operational",
        "timestamp":       time.time(),
        "active_missions": len(active_missions),
        "ws_clients":      len(manager.connections),
    }


@app.get("/api/system/status")
async def system_status():
    return SystemStatus(
        active_drones=len(active_missions),
        active_missions=len(active_missions),
        total_flights_today=stats["total_missions"],
        conflicts_detected=stats["conflicts_detected"],
        conflicts_resolved=stats["conflicts_resolved"],
        system_health="operational",
        timestamp=time.time(),
    )


@app.get("/api/geofencing/zones")
async def get_zones():
    return geofencing.get_geofence_info()


# ══════════════════════════════════════════════════════════════════════════════
# ROUTES — AIRSPACE STATUS
# ══════════════════════════════════════════════════════════════════════════════

@app.get("/api/airspace/status")
async def airspace_status():
    """
    Debug endpoint: shows all approved 4-D flight plans and their strata.
    Useful for verifying the conflict resolution is working correctly.
    """
    plans = []
    for drone_id, t4d in flight_plans_4d.items():
        mission = next(
            (m for m in active_missions.values() if m.drone_id == drone_id), None
        )
        first_wp = t4d.waypoints[0]  if t4d.waypoints else None
        last_wp  = t4d.waypoints[-1] if t4d.waypoints else None
        plans.append({
            "drone_id":       drone_id,
            "mission_id":     mission.mission_id if mission else None,
            "stratum_m":      t4d.stratum,
            "direction":      t4d.direction,
            "takeoff_delay_s": t4d.takeoff_delay,
            "start_eta":      first_wp.eta if first_wp else None,
            "end_eta":        last_wp.eta  if last_wp  else None,
            "waypoints":      len(t4d.waypoints),
            "total_dist_km":  round(t4d.total_distance / 1000, 2),
        })

    return {
        "timestamp":       time.time(),
        "active_flights":  len(plans),
        "flight_plans":    plans,
        "pad_queue_keys":  list(conflict_detection.pad_queue._slots.keys()),
    }


# ══════════════════════════════════════════════════════════════════════════════
# ROUTES — MISSIONS
# ══════════════════════════════════════════════════════════════════════════════

@app.get("/api/missions")
async def get_missions():
    return list(active_missions.values())


@app.get("/api/missions/{mission_id}")
async def get_mission(mission_id: str):
    if mission_id not in active_missions:
        raise HTTPException(404, "Mission not found")
    return active_missions[mission_id]


@app.delete("/api/missions/{mission_id}")
async def cancel_mission(mission_id: str):
    """
    Cancel an in-flight mission immediately.
    Releases pad reservations and removes the drone from conflict checks.
    The frontend will clean up the entity via the mission_cancelled broadcast.
    """
    if mission_id not in active_missions:
        raise HTTPException(404, "Mission not found")

    mission  = active_missions[mission_id]
    drone_id = mission.drone_id

    active_missions.pop(mission_id, None)
    flight_plans_4d.pop(drone_id, None)
    flight_plans_legacy.pop(drone_id, None)
    conflict_detection.pad_queue.release(drone_id)

    await manager.broadcast({
        "type":       "mission_cancelled",
        "mission_id": mission_id,
        "drone_id":   drone_id,
    })

    print(f"[{mission_id}] Cancelled by operator")
    return {"status": "cancelled", "mission_id": mission_id}


# ══════════════════════════════════════════════════════════════════════════════
# ROUTES — DELIVERY REQUEST  (main planning pipeline)
# ══════════════════════════════════════════════════════════════════════════════

@app.post("/api/delivery/request")
async def create_delivery(request: DeliveryRequest):
    """
    Plan and approve a delivery mission with full 4-D conflict resolution.

    Steps:
      1  Validate inputs (area bounds, no-fly zones, water bodies)
      2  Straight-line path → reroute animation data
      3  Plan 4-D trajectory (OSRM roads + climb/descent + sigma_t)
      4  Reserve pad slots  (FIFO queue — may add pad_delay)
      5  CPA conflict cascade  (altitude → takeoff delay)
      6  Re-reserve pad slots with final ETAs  (post-CPA)
      7  Build and store mission
      8  Broadcast mission_created
      9  Schedule cleanup task
    """

    # ── Step 1: Input validation ──────────────────────────────────────────────
    if not geofencing.is_within_operational_area(request.pickup):
        raise HTTPException(400, "Pickup location is outside the operational area")
    if not geofencing.is_within_operational_area(request.delivery):
        raise HTTPException(400, "Delivery location is outside the operational area")

    if geofencing.is_in_no_fly_zone(request.pickup):
        raise HTTPException(400, "Pickup location is inside a no-fly zone")
    if geofencing.is_in_no_fly_zone(request.delivery):
        raise HTTPException(400, "Delivery location is inside a no-fly zone")

    pickup_water = geofencing.is_over_water(
        request.pickup.latitude, request.pickup.longitude)
    if pickup_water:
        raise HTTPException(
            400, f"Pickup location is over {pickup_water} — choose a point on land")
    delivery_water = geofencing.is_over_water(
        request.delivery.latitude, request.delivery.longitude)
    if delivery_water:
        raise HTTPException(
            400, f"Delivery location is over {delivery_water} — choose a point on land")

    # ── Step 2: IDs + reroute viz ─────────────────────────────────────────────
    mission_id = f"mission_{uuid.uuid4().hex[:8]}"
    drone_id   = f"drone_{uuid.uuid4().hex[:6]}"

    print(f"\n[{mission_id}] New delivery → drone {drone_id}")
    print(f"  Pickup:   ({request.pickup.latitude:.4f},  {request.pickup.longitude:.4f})")
    print(f"  Delivery: ({request.delivery.latitude:.4f}, {request.delivery.longitude:.4f})")

    # Straight-line path is pure math — no thread needed
    direct_path   = pathfinding.get_direct_path(request.pickup, request.delivery)
    was_rerouted  = pathfinding._path_hits_no_fly_zone(direct_path)
    blocked_path_json = [[p[0], p[1]] for p in direct_path] if was_rerouted else None

    # ── Step 3: Plan 4-D trajectory ───────────────────────────────────────────
    # plan_trajectory() now returns (primary_traj, alternatives) where
    # alternatives is a list of up to K−1 pre-built Yen's trajectories at
    # full speed, sorted by travel time.  The conflict resolver uses them
    # for Layer-2 Option B (Path vs. Speed trade-off).
    # OSRM HTTP calls are blocking → thread pool.
    plan_result: Optional[tuple] = await asyncio.to_thread(
        pathfinding.plan_trajectory,
        request.pickup, request.delivery, time.time(),
    )
    if plan_result is None:
        raise HTTPException(500, "Could not find a valid flight path between those locations")

    traj4d, traj4d_alternatives = plan_result

    print(f"[{mission_id}] ✓ Trajectory planned:"
          f"  {len(traj4d.waypoints)} wps"
          f"  | {traj4d.total_distance / 1000:.2f} km"
          f"  | stratum {traj4d.stratum} m ({traj4d.direction})"
          f"  | ETA {traj4d.total_time / 60:.1f} min"
          + ("  | REROUTED" if was_rerouted else ""))

    # ── Step 4: Pad queue (conditionally bypassed) ────────────────────────────
    pickup_lat   = request.pickup.latitude
    pickup_lon   = request.pickup.longitude
    delivery_lat = request.delivery.latitude
    delivery_lon = request.delivery.longitude

    if conflict_detection.PAD_QUEUE_ENABLED:
        # Original FIFO pad logic — enforces sequential pad use
        earliest_takeoff = conflict_detection.pad_queue.earliest_available(
            pickup_lat, pickup_lon, not_before=time.time()
        )
        pad_delay = max(0.0, earliest_takeoff - time.time())
        if pad_delay > 0.0:
            print(f"[{mission_id}] Pickup pad busy — delaying +{pad_delay:.1f}s")
            traj4d = conflict_detection.apply_delay(traj4d, pad_delay)

        landing_eta   = traj4d.waypoints[-1].eta
        landing_start = landing_eta - config.PAD_CLEARANCE_TIME
        earliest_land = conflict_detection.pad_queue.earliest_available(
            delivery_lat, delivery_lon, not_before=landing_start
        )
        if earliest_land > landing_start:
            extra = earliest_land - landing_start
            print(f"[{mission_id}] Delivery pad busy — delaying +{extra:.1f}s")
            traj4d       = conflict_detection.apply_delay(traj4d, extra)
            landing_eta  = traj4d.waypoints[-1].eta
            landing_start = landing_eta - config.PAD_CLEARANCE_TIME

        takeoff_start = traj4d.waypoints[0].eta
        takeoff_end   = takeoff_start + config.PAD_CLEARANCE_TIME
        conflict_detection.pad_queue.reserve(
            drone_id, pickup_lat, pickup_lon, takeoff_start, takeoff_end)
        conflict_detection.pad_queue.reserve(
            drone_id, delivery_lat, delivery_lon, landing_start, landing_eta)
    else:
        # Pad queue bypassed — all drones launch immediately.
        # CPA conflict resolution (Step 5) still runs and will detect
        # mid-air 4D overlaps, triggering altitude strata changes or
        # takeoff delays as needed.
        print(f"[{mission_id}] Pad queue bypassed — immediate launch")

    # ── Step 5: CPA conflict cascade (Layer 1: altitude → Layer 2: path/speed) ─
    alert: Optional[ConflictAlert] = None
    resolution_label: str = 'NO_CONFLICT'
    try:
        # Extract the 2-D road path from the primary trajectory so the
        # resolver can rebuild at different strata / speeds without re-running
        # OSRM.  GROUND waypoints are excluded (they sit on the pad at z=0).
        latlon_path = [
            (wp.position.latitude, wp.position.longitude)
            for wp in traj4d.waypoints
            if wp.phase.name != 'GROUND'
        ]

        resolved, resolution_label = await asyncio.to_thread(
            conflict_detection.resolve_conflict,
            drone_id,
            traj4d,
            traj4d_alternatives,
            flight_plans_4d,
            request.pickup,
            request.delivery,
            latlon_path,
        )
        traj4d = resolved   # always use the (possibly unchanged) resolved traj

        if resolution_label != 'NO_CONFLICT':
            stats["conflicts_detected"] += 1
            stats["conflicts_resolved"] += 1
            alert = ConflictAlert(
                drone_id=drone_id,
                conflicting_drone_id="detected",
                conflict_time=time.time(),
                conflict_position=traj4d.waypoints[0].position,
                resolution_action=resolution_label,
                new_stratum=traj4d.stratum,
                delay_seconds=traj4d.takeoff_delay,
            )
            print(f"[{mission_id}] ✓ Conflict resolved:"
                  f"  method={resolution_label}"
                  f"  delay={traj4d.takeoff_delay:.1f}s"
                  f"  stratum={traj4d.stratum}m")
            await manager.broadcast({
                "type":              "conflict_resolved",
                "conflict":          alert.model_dump(),
                "resolution_method": resolution_label,
                "takeoff_delay_s":   traj4d.takeoff_delay,
                "final_stratum_m":   traj4d.stratum,
            })

    except conflict_detection.AirspaceSaturatedError as exc:
        # Rollback pad reservations before rejecting
        conflict_detection.pad_queue.release(drone_id)
        raise HTTPException(
            status_code=503,
            detail={
                "error":       "airspace_saturated",
                "message":     str(exc),
                "retry_after": exc.retry_after,
                "conflicts":   exc.conflicts,
            },
        )

    # ── WLS state reset on stratum change (Layer 1) ───────────────────────────
    # When the resolver changes the stratum, anchor expected_dist_m values in
    # the rebuilt trajectory are recomputed for the new altitude.  Clearing the
    # per-drone WLS state ensures the GPS-denied module starts clean and doesn't
    # carry over residuals from the old altitude.
    if resolution_label.startswith('LAYER1_STRATUM'):
        _wls_state.pop(drone_id, None)
        _wls_wp_idx.pop(drone_id, None)
        _wls_active_nodes.pop(drone_id, None)
        print(f"[{mission_id}] WLS state reset — stratum changed to {traj4d.stratum}m")
    # ── Step 6: Re-reserve pad slots with final ETAs ──────────────────────────
    if conflict_detection.PAD_QUEUE_ENABLED:
        conflict_detection.pad_queue.release(drone_id)

        final_takeoff_start = traj4d.waypoints[0].eta
        final_takeoff_end   = final_takeoff_start + config.PAD_CLEARANCE_TIME
        final_landing_eta   = traj4d.waypoints[-1].eta
        final_landing_start = final_landing_eta - config.PAD_CLEARANCE_TIME

        conflict_detection.pad_queue.reserve(
            drone_id, pickup_lat, pickup_lon,
            final_takeoff_start, final_takeoff_end)
        conflict_detection.pad_queue.reserve(
            drone_id, delivery_lat, delivery_lon,
            final_landing_start, final_landing_eta)

    # ── Step 7: Build mission ─────────────────────────────────────────────────
    # Convert 4-D trajectory to the legacy format the frontend expects.
    # Store both: 4-D for CPA checks, legacy for telemetry interpolation.
    legacy_traj: Trajectory = traj4d.to_legacy()

    mission = Mission(
        mission_id=mission_id,
        drone_id=drone_id,
        pickup_location=request.pickup,
        delivery_location=request.delivery,
        created_at=time.time(),
        assigned_at=time.time(),
        status=DroneStatus.EN_ROUTE_DELIVERY,
        trajectory=legacy_traj,
    )

    active_missions[mission_id]     = mission
    flight_plans_4d[drone_id]       = traj4d
    flight_plans_legacy[drone_id]   = legacy_traj
    stats["total_missions"]        += 1

    # ── Step 8: Broadcast ─────────────────────────────────────────────────────
    await manager.broadcast({
        "type":          "mission_created",
        "mission":       mission.model_dump(),
        "blocked_path":  blocked_path_json,
        "stratum_m":     traj4d.stratum,
        "direction":     traj4d.direction,
        "takeoff_delay_s": traj4d.takeoff_delay,
    })

    # ── Step 9: Auto-cleanup ──────────────────────────────────────────────────
    # flight_duration_s is relative to NOW, using the final baked ETAs.
    flight_duration_s = legacy_traj.waypoints[-1].eta - time.time()
    asyncio.create_task(
        _cleanup_mission(mission_id, drone_id, max(0.0, flight_duration_s) + 5.0)
    )

    return {
        "mission_id":        mission_id,
        "drone_id":          drone_id,
        "status":            "en_route",
        "trajectory":        legacy_traj.model_dump(),
        "stratum_m":         traj4d.stratum,
        "direction":         traj4d.direction,
        "takeoff_delay_s":   traj4d.takeoff_delay,
        "conflict_resolved": alert is not None,
        "total_dist_km":     round(traj4d.total_distance / 1000, 2),
        "eta_minutes":       round(traj4d.total_time / 60, 1),
    }


# ══════════════════════════════════════════════════════════════════════════════
# CLEANUP
# ══════════════════════════════════════════════════════════════════════════════

async def _cleanup_mission(mission_id: str, drone_id: str, delay_s: float) -> None:
    """
    After the drone lands + a small grace period:
      - Remove from active_missions and flight_plans
      - Release pad queue slots
      - Broadcast mission_complete so the frontend can remove the drone entity
    """
    await asyncio.sleep(delay_s)

    active_missions.pop(mission_id, None)
    flight_plans_4d.pop(drone_id, None)
    flight_plans_legacy.pop(drone_id, None)
    conflict_detection.pad_queue.release(drone_id)

    await manager.broadcast({
        "type":       "mission_complete",
        "mission_id": mission_id,
        "drone_id":   drone_id,
    })

    print(f"[{mission_id}] ✓ Complete — drone {drone_id} retired")


# ══════════════════════════════════════════════════════════════════════════════
# WEBSOCKET  (telemetry stream + initial state push)
# ══════════════════════════════════════════════════════════════════════════════

async def _ws_keepalive(ws: WebSocket) -> None:
    """Send a ping every 20 s to prevent idle connection timeout."""
    while True:
        await asyncio.sleep(20)
        try:
            await ws.send_json({"type": "ping", "timestamp": time.time()})
        except Exception:
            break


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await manager.connect(ws)

    # Push current geofencing config immediately so the client can draw zones.
    # Also push the anchor node registry so the frontend has a single source of
    # truth — no hardcoded coordinates in app.js.
    await manager.send_to(ws, {
        "type":       "initial_state",
        "geofencing": geofencing.get_geofence_info(),
        "anchor_nodes": [
            {
                "node_id": node.node_id,
                "label":   node.label,
                "lat":     node.lat,
                "lon":     node.lon,
                "alt":     node.alt_m,
            }
            for node in anchor_registry.get_anchor_registry()
        ],
        "stats": {
            "total_missions":     stats["total_missions"],
            "conflicts_detected": stats["conflicts_detected"],
            "conflicts_resolved": stats["conflicts_resolved"],
        },
    })

    # Push all currently active missions so late-joining clients are in sync
    if active_missions:
        for m in active_missions.values():
            traj4d = flight_plans_4d.get(m.drone_id)
            await manager.send_to(ws, {
                "type":      "mission_created",
                "mission":   m.model_dump(),
                "blocked_path": None,
                "stratum_m": traj4d.stratum   if traj4d else None,
                "direction": traj4d.direction if traj4d else None,
                "takeoff_delay_s": traj4d.takeoff_delay if traj4d else 0,
            })

    keepalive_task = asyncio.create_task(_ws_keepalive(ws))

    try:
        while True:
            # We don't expect client messages, but receive() keeps the conn alive
            await ws.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        keepalive_task.cancel()
        manager.disconnect(ws)


if __name__ == "__main__":
    import uvicorn
    # This acts as your "main()" function call
    uvicorn.run(app, host="127.0.0.1", port=8000)
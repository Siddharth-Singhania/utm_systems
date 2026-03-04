# UTM System — End-to-End Flow

> Documents the complete chain of function calls triggered by a single delivery request, from the moment a user clicks "Request Delivery" to the moment a drone lands and is cleaned up.

---

## Overview Diagram

```
Browser (app.js)
    │  POST /api/delivery/request
    ▼
main.py  create_delivery()
    │
    ├─── geofencing.py          [Validate inputs]
    │
    ├─── pathfinding.py         [Plan 2D → 4D trajectory]
    │        │
    │        ├── OSRM API       [Road route]
    │        ├── geofencing.py  [Zone validation]
    │        ├── anchors.py     [Anchor bindings per waypoint]
    │        └── visibility_graph_astar()  [Yen's K=5 bypass]
    │
    ├─── conflict_detection.py  [CPA check + resolve]
    │        │
    │        └── reassign_stratum() / apply_delay()
    │                 │
    │                 └── anchors.py  [Recompute bindings at new altitude]
    │
    ├─── WebSocket broadcast    [mission_created → Cesium spawns drone]
    │
    └─── _telemetry_loop()      [2 Hz position updates]
             │
             ├── gps_denied.py  [WLS correction at each waypoint]
             │        │
             │        └── anchors.py  [Node registry for distance calc]
             │
             └── WebSocket broadcast  [telemetry → Cesium moves drone]
```

---

## Stage 1 — The Planning Phase

### 1.1 Frontend → Backend

The user fills in pickup/delivery coordinates and clicks **Request Delivery**. `app.js` calls `createDelivery()`, which first runs the client-side `isOverWater()` check (a JS mirror of `geofencing.is_over_water`) to give instant feedback. If the coordinates pass, it fires:

```
POST /api/delivery/request
Body: { pickup: {lat, lon, alt}, delivery: {lat, lon, alt} }
```

This arrives at `main.py → create_delivery()`.

---

### 1.2 Input Validation (`main.py` → `geofencing.py`)

`create_delivery()` immediately calls four guard functions before touching the pathfinder:

```
main.py  create_delivery()
  │
  ├── geofencing.is_within_operational_area(pickup)
  ├── geofencing.is_within_operational_area(delivery)
  ├── geofencing.is_in_no_fly_zone(pickup)
  ├── geofencing.is_in_no_fly_zone(delivery)
  ├── geofencing.is_over_water(pickup.lat, pickup.lon)
  └── geofencing.is_over_water(delivery.lat, delivery.lon)
```

**Why these calls happen here:** `main.py` cannot know the geographic rules — all spatial knowledge lives in `geofencing.py`. If any check fails, a `400 HTTPException` is raised immediately and no trajectory computation is triggered.

---

### 1.3 Reroute Animation Data

```
main.py  create_delivery()
  │
  └── pathfinding.get_direct_path(pickup, delivery)
        └── returns [(pickup.lat, pickup.lon), (delivery.lat, delivery.lon)]

  └── pathfinding._path_hits_no_fly_zone(direct_path)
        └── geofencing.point_in_polygon() for each zone
              └── returns True → blocked_path_json is set
```

This straight-line path is stored as `blocked_path_json` and will be broadcast alongside the real route so the Cesium frontend can play the red-flash reroute animation. It is computed synchronously because it is pure arithmetic.

---

### 1.4 4D Trajectory Planning (`main.py` → `pathfinding.py` → `geofencing.py` + `anchors.py` + OSRM)

```
main.py  create_delivery()
  │
  └── asyncio.to_thread(pathfinding.plan_trajectory, pickup, delivery, now)
```

The call is dispatched to a thread pool because `plan_trajectory` makes blocking OSRM HTTP requests. Inside:

```
pathfinding.plan_trajectory()
  │
  ├── fetch_osrm_route(start, goal)
  │     └── HTTP GET router.project-osrm.org
  │           Returns List[LatLon] — real road geometry
  │
  ├── _path_hits_no_fly_zone(road_points)
  │     └── geofencing.point_in_polygon() per segment per zone
  │           [Why: OSRM doesn't know about UTM no-fly zones]
  │
  │   ── If road is CLEAR:
  │       latlon_path = road_points  ✓  (Level 1)
  │
  │   ── If road is BLOCKED:
  │       visibility_graph_astar(start, goal)  (Yen's K=5)
  │         │
  │         ├── _buffered_vertices(zone.polygon)
  │         │     [Build bypass node set with safety clearance]
  │         │
  │         ├── _astar_on_graph(nodes, blocked_edges={}, blocked_nodes={})
  │         │     [Find base path A[0] by weighted edge cost]
  │         │
  │         ├── Yen's loop (k=1..4):
  │         │     For each spur_node in A[k-1]:
  │         │       Block edges used by confirmed paths with same root
  │         │       _astar_on_graph(nodes, blocked_edges, blocked_nodes)
  │         │       If found → add root+spur to candidate heap
  │         │     A[k] = pop best candidate
  │         │
  │         └── Score all A[0..k] with get_path_travel_time()
  │               → return time-optimal path
  │
  │       snap_to_road(bypass_corner)  [Level 2: pull corners to roads]
  │       fetch_osrm_route_via_positions([start]+snapped+[goal])
  │           [Chain a real road route through the bypass points]
  │
  │   ── If OSRM unavailable:
  │       visibility_graph_astar()  (Level 3 — pure visibility graph)
  │
  └── build_4d_trajectory(latlon_path, pickup, delivery, stratum, direction, now)
        │
        ├── bearing_to_direction(overall_heading) → "NORTH" / "EAST" / etc.
        ├── direction_to_stratum(direction)        → cruise altitude metres
        │
        ├── walk_path_forward()  → climb endpoint
        ├── walk_path_backward() → descent start point
        │
        ├── For each segment (GROUND→CLIMB→CRUISE→DESCENT→GROUND):
        │     calculate_heading()
        │     _turn_speed()        [CRUISE: slow on corners]
        │     seg_time = dist / speed
        │     _compute_sigma_t()   [timing uncertainty grows with arc distance]
        │     │
        │     └── anchor_registry.compute_anchor_bindings(wp_xyz)
        │               │
        │               ├── select_anchors(drone_xyz)
        │               │     ├── range filter against _REGISTRY (1km grid)
        │               │     └── greedy GDOP minimisation (max 6 nodes)
        │               └── returns List[AnchorBinding] + hdop + vdop
        │                     [Stored in Waypoint4D for later GPS-denied use]
        │
        └── returns Trajectory4D
              waypoints: List[Waypoint4D]
                each with: position, eta, speed, heading,
                           phase, sigma_t, anchor_bindings
```

**Why `anchors.py` is called here, not during flight:** The expected distances to each node are computed from the *planned* position at planning time. This gives `gps_denied.py` a reference baseline — the difference between these planned distances and the measured distances during flight is exactly the drift signal.

---

## Stage 2 — The Conflict Resolution Loop

### 2.1 Pad Queue (Optional)

```
main.py  create_delivery()
  │
  └── if PAD_QUEUE_ENABLED:
        conflict_detection.pad_queue.earliest_available(pickup_lat, pickup_lon, now)
          [Returns first free time slot on the pickup pad]
        → apply_delay(traj4d, pad_delay) if pad is busy
        conflict_detection.pad_queue.reserve(drone_id, ...)
```

With `PAD_QUEUE_ENABLED = False` (conflict demo mode), this block is skipped entirely and every drone launches at `time.time()`. This guarantees 4D path overlaps for the conflict visualisation demos.

---

### 2.2 The CPA Cascade — Check-Fix-Recheck

```
main.py  create_delivery()
  │
  └── asyncio.to_thread(resolver.resolve, traj4d, flight_plans_4d, pickup, delivery)
```

Again dispatched to a thread pool because the cascade loop can involve multiple GDOP passes via `reassign_stratum`.

```
conflict_detection.ConflictResolver.resolve()
  │
  └── LOOP (up to MAX_RESOLUTION_PASSES = 5):
        │
        ├── check_trajectory_vs_all(candidate, existing_flight_plans)
        │     │
        │     └── For every (candidate_segment, existing_segment) pair
        │           with overlapping time windows:
        │             _segment_cpa(a0, a1, ta0, ta1, b0, b1, tb0, tb1)
        │               [Analytic quadratic CPA — no time-sampling blind spots]
        │             _effective_radius(wp_a, wp_b)
        │               [R_eff = R_BASE + speed_A×σ_A + speed_B×σ_B]
        │             If d_h < R_eff AND d_v < CPA_V_SEP → ConflictInfo
        │
        ├── If conflicts == [] → DONE ✓
        │
        ├── Sort conflicts by earliest CPA time
        │   Take worst conflict (drone_id, cpa_time, severity)
        │
        ├── RESOLUTION 1 — Try alternate altitude stratum:
        │     For each stratum in ALL_STRATA_ORDERED (nearest to current first):
        │       reassign_stratum(traj, new_stratum, pickup, delivery)
        │         │
        │         └── For every waypoint at new altitude:
        │               anchor_registry.compute_anchor_bindings(wp_xyz)
        │               [CRITICAL: recompute, not copy — old expected_dist_m
        │                would bias WLS residuals at new altitude]
        │         │
        │         └── check_trajectory_vs_all(candidate_alt, existing)
        │               If clear → adopt new stratum, SET alert, CONTINUE loop
        │
        └── RESOLUTION 2 — Binary-search takeoff delay:
              (only if all strata are blocked)
              combined_sigma = traj.wp[0].sigma_t + conflict_drone.wp[0].sigma_t
              lo = combined_sigma + TEMPORAL_SAFETY_MARGIN
              hi = MAX_TAKEOFF_DELAY - total_delay_already_applied
              │
              └── _binary_search_delay(traj, existing, lo, hi)
                    8 iterations of binary search:
                      mid = (lo+hi)/2
                      check_trajectory_vs_all(apply_delay(traj, mid), existing)
                      If conflicts → lo = mid (need more delay)
                      Else         → hi = mid (try less)
                    Returns smallest clean delay
                  apply_delay(traj, best_delay) → shift all ETAs
```

**Why the loop repeats:** Fixing one conflict can create another. If drone A is bumped from stratum 60 m to 80 m, it might now conflict with drone C that was already at 80 m. The re-validation pass catches this. The system only stops when `check_trajectory_vs_all` returns an empty list.

**Why stratum is tried before delay:** Altitude separation is free — it doesn't make the delivery late. A takeoff delay is always the last resort because it directly increases the customer's wait time.

---

### 2.3 Post-Resolution Steps

```
main.py  create_delivery()
  │
  ├── if PAD_QUEUE_ENABLED:
  │     pad_queue.release(drone_id)   [Drop tentative reservations]
  │     pad_queue.reserve(...) with FINAL ETAs from resolved trajectory
  │
  ├── traj4d.to_legacy() → Trajectory
  │     [Strip 4D-only fields; frontend only needs position+eta+speed+heading]
  │
  ├── active_missions[mission_id]   = mission
  ├── flight_plans_4d[drone_id]     = traj4d    [Used for future CPA checks]
  ├── flight_plans_legacy[drone_id] = legacy    [Used for telemetry interpolation]
  │
  └── manager.broadcast({ type: "mission_created", mission, blocked_path, ... })
        [WebSocket → app.js → spawnMissionDrone() → Cesium entity created]
```

---

## Stage 3 — Execution and GPS-Denied Simulation

### 3.1 The Telemetry Loop (`main.py → gps_denied.py → anchors.py`)

The `_telemetry_loop()` coroutine runs continuously at 2 Hz (every 0.5 s) for the lifetime of the server.

```
main.py  _telemetry_loop()  [fires every 0.5s]
  │
  ├── For each active mission:
  │     │
  │     ├── _interpolate_position(traj_legacy, now)
  │     │     [Linear interpolation along planned waypoints by wall-clock time]
  │     │     [This is the primary position driver — trajectory controls movement]
  │     │
  │     ├── Checkpoint detection:
  │     │     current_wp_idx = last Waypoint4D whose eta ≤ now
  │     │     if current_wp_idx > prev_wp_idx:
  │     │       → New waypoint reached → trigger WLS fix
  │     │
  │     ├── [WLS FIX — at each new waypoint]
  │     │     wp_now = traj4d.waypoints[current_wp_idx]
  │     │     planned_xyz = anchor_registry._to_xyz(wp_now.position)
  │     │     │
  │     │     ├── Simulate IMU drift:
  │     │     │     true_xyz = planned_xyz + Gaussian(0, 5m) per axis
  │     │     │     [Represents accumulated inertial error since last fix]
  │     │     │
  │     │     ├── gps_denied.simulate_ranging(wp_now.anchor_bindings, true_xyz)
  │     │     │     │
  │     │     │     └── For each AnchorBinding in the waypoint:
  │     │     │           node = _NODE_MAP[binding.node_id]
  │     │     │           true_dist = ||node.xyz − true_xyz||
  │     │     │           σ = UWB_SIGMA_0 + UWB_DRIFT_K × true_dist
  │     │     │           measured = true_dist + N(0, σ)
  │     │     │           returns [(node_id, expected_dist, measured_dist)]
  │     │     │           [expected_dist came from anchors.py at planning time]
  │     │     │
  │     │     └── gps_denied.compute_wls_correction(
  │     │               wp_now.anchor_bindings, measurements, planned_xyz)
  │     │           │
  │     │           ├── _build_H_and_residuals()
  │     │           │     r_i = measured_i − expected_i  (raw range error)
  │     │           │     û_i = unit vector planned→node
  │     │           │     w_i = 1/σ_i²
  │     │           │
  │     │           ├── _gdop_from_H() → hdop, vdop
  │     │           │     If hdop > HDOP_THRESHOLD → zero out Δx, Δy
  │     │           │     If vdop > VDOP_THRESHOLD → zero out Δz
  │     │           │
  │     │           ├── _solve_wls()
  │     │           │     Δp = −(HᵀWH)⁻¹ · Hᵀ · W · r
  │     │           │
  │     │           └── returns WLSResult(delta_xyz, velocity_correction,
  │     │                                 hdop_ok, vdop_ok, n_nodes, rms)
  │     │
  │     │     Persist: _wls_state[drone_id]       = WLSResult
  │     │              _wls_active_nodes[drone_id] = [node_ids from bindings]
  │     │
  │     ├── [CORRECTION APPLICATION — within WLS_CORRECTION_TAU window]
  │     │     time_since_wp = now − wp_now.eta
  │     │     if 0 ≤ time_since_wp ≤ WLS_CORRECTION_TAU:
  │     │       fade = 1 − (time_since_wp / WLS_CORRECTION_TAU)
  │     │       correction = delta_xyz × fade
  │     │       [Linearly fades the correction to zero over τ seconds]
  │     │
  │     ├── Apply to geographic position:
  │     │     corrected_lat = planned_lat + Δx / LAT_M
  │     │     corrected_lon = planned_lon + Δy / (LON_M × cos(lat))
  │     │     corrected_alt = planned_alt + Δz
  │     │
  │     └── manager.broadcast({ type: "telemetry",
  │               position: corrected,
  │               battery_level: _estimate_battery(...),
  │               gps_denied: {
  │                 anchor_nodes_used: n_nodes,
  │                 hdop_ok, vdop_ok,
  │                 correction_m: |Δp|,
  │                 residual_rms,
  │                 active_node_ids   ← persisted from last WLS, not cleared
  │               }
  │         })
  │
  └── app.js receives telemetry
        latestTelemetry.set(drone_id, data)
        updateRangingLines()  → green dashed lines tower→drone in Cesium
        refreshInfoPanel()    → HDOP/VDOP bars, correction magnitude, RMS
```

**Why `active_node_ids` is persisted across ticks:** The WLS fix fires only once per waypoint (~every 5–30 s depending on speed), but telemetry arrives every 0.5 s. If `active_node_ids` were cleared between waypoints, the ranging lines would flash on and off 10–60 times per waypoint. Persisting the last-fired node set keeps the lines stable.

**Why the correction fades to zero:** The WLS gives a single-shot position fix. Between waypoint checkpoints the drone is dead-reckoning (following the plan by time). Fading the correction over `WLS_CORRECTION_TAU` seconds blends the fix smoothly rather than applying a sudden jump.

---

### 3.2 Frontend Rendering (`app.js`)

```
app.js  handleWebSocketMessage()
  │
  ├── type: "initial_state"
  │     loadGeofencing()        → Cesium red/orange zone polygons
  │     initAnchorTowers()      → orange tower billboards + shaft polylines
  │
  ├── type: "mission_created"
  │     spawnMissionDrone()
  │       SampledPositionProperty ← all Waypoint ETAs baked in
  │       Lagrange interpolation  ← smooth drone movement
  │       routeEntity (hidden)    ← shown on click or toggle
  │       reroute animation       ← red dashed blocked path for 2.5s
  │
  └── type: "telemetry"
        latestTelemetry.set(drone_id, data)
        if gps_denied.active:
          updateRangingLines()
            _getOrCreateLine(drone_id, node_id)
              PolylineCollection primitive (not entity — avoids terrain clipping)
            line.positions = [drone_cartesian, node_tip_cartesian]
            line.show = true
        if selectedMission == this drone:
          refreshInfoPanel()
            HDOP/VDOP bars, anchor count badge, correction magnitude, RMS colour
```

---

### 3.3 Mission Cleanup

```
main.py  _cleanup_mission(mission_id, drone_id, delay_s)
  │                            [fires delay_s after last waypoint ETA]
  │
  ├── active_missions.pop(mission_id)
  ├── flight_plans_4d.pop(drone_id)     [removes from future CPA checks]
  ├── flight_plans_legacy.pop(drone_id)
  ├── pad_queue.release(drone_id)
  │
  └── manager.broadcast({ type: "mission_complete", mission_id, drone_id })
        │
        └── app.js  cleanupMission(missionId)
              viewer.entities.remove(droneEntity)
              viewer.entities.remove(routeEntity)
              _removeAllLinesForDrone(droneId)
                _uwbLineCollection.remove(line) for each node line
              droneIdToMission.delete(droneId)
              lastRangingData.delete(droneId)
```

**Why `_removeAllLinesForDrone` explicitly removes from `_uwbLineCollection`:** Cesium's `PolylineCollection` does not garbage-collect removed entries automatically. Entities set to `show = false` still consume memory. Explicit removal prevents ghost primitives accumulating across a long session.

---

## Key Design Principles

| Principle | Where | Why |
|-----------|-------|-----|
| All conflict resolution happens at planning time | `conflict_detection.py` | Drones have no runtime communication — once airborne the plan is fixed |
| Anchor bindings are recomputed on stratum change | `reassign_stratum()` | Expected distances are altitude-dependent; stale values corrupt WLS residuals |
| Pad queue is bypassed for conflict demos | `PAD_QUEUE_ENABLED = False` | Simultaneous launches create guaranteed 4D overlaps for visualisation |
| Position = trajectory + WLS correction | `_telemetry_loop()` | Trajectory drives movement; towers correct accumulated drift (mirrors real INS/UWB) |
| OSRM is called in a thread pool | `asyncio.to_thread(plan_trajectory)` | Blocking HTTP calls would stall the async event loop and freeze all WebSocket clients |
| Yen's K=5 scores by time, not distance | `visibility_graph_astar()` | A shorter path with many sharp corners can be slower than a slightly longer straight one |
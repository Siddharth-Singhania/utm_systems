"""
4-D Trajectory-Based Conflict Resolution
=========================================

Architecture
------------
All conflict resolution happens at PLANNING TIME, before the drone launches.
Adjustments are baked into the Trajectory4D before it is broadcast.

Three protection layers, applied in order:
  Layer 0 - Structural (direction-exclusive altitude strata)
             Free separation, handled in pathfinding.py before this module.
  Layer 1 - Vertical separation (switch to a different stratum)
  Layer 2 - Temporal separation (uniform takeoff delay)

Conflict detection uses CONTINUOUS CLOSEST-POINT-OF-APPROACH (CPA) math
on every pair of trajectory segments whose time windows overlap.
This eliminates the blind spots of discrete time-sampling.

The effective protection radius expands with timing uncertainty (sigma_t):
    R_eff(wp) = R_BASE + speed * sigma_t(wp)

Terminal areas (r < TERMINAL_RADIUS from a pad, alt < TERMINAL_HEIGHT) use
a FIFO pad queue rather than CPA checks.
"""

import math
import time
from typing import Dict, List, Optional, Tuple
from models import (
    Position, ConflictAlert, Trajectory4D, Waypoint4D, FlightPhase
)
import config
import uuid

import anchors as anchor_registry
# ── Type aliases ──────────────────────────────────────────────────────────────
Vec3 = Tuple[float, float, float]   # (lat_m, lon_m, alt_m) in metres
PAD_QUEUE_ENABLED = False

# ── Coordinate helpers ────────────────────────────────────────────────────────
_LAT_M  = 111_319.5   # metres per degree of latitude
_LON_M  = 111_319.5   # metres per degree of longitude at equator


def _to_metres(lat: float, lon: float, alt: float,
               ref_lat: float = 37.75) -> Vec3:
    """Convert (lat, lon, alt) to a local Cartesian metric frame.
    Uses a flat-earth projection centred at ref_lat.
    Accurate to < 0.1 % over the SF operational area (~40 km).
    """
    x = (lat - ref_lat) * _LAT_M
    y = (lon - (-122.42)) * _LON_M * math.cos(math.radians(ref_lat))
    z = alt
    return (x, y, z)


def _dist3(a: Vec3, b: Vec3) -> float:
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)


def _dist2(a: Vec3, b: Vec3) -> float:
    """Horizontal-only distance (ignores z)."""
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)


# ══════════════════════════════════════════════════════════════════════════════
# PAD QUEUE  (Terminal Area FIFO)
# ══════════════════════════════════════════════════════════════════════════════

class PadQueue:
    """
    FIFO time-slot queue for each launch/landing pad.

    Pad identity is a rounded (lat, lon) key (1-decimal-place precision,
    ~10 km grid) — pads within ~5 km are treated as the same pad for
    terminal-area conflict purposes.
    """
    # Coarser rounding for pad identity
    _ROUND = 2   # 2 decimal places ≈ 1.1 km grid

    def __init__(self) -> None:
        # pad_key -> list of (drone_id, window_start, window_end) sorted by window_start
        self._slots: Dict[str, List[Tuple[str, float, float]]] = {}

    @staticmethod
    def _key(lat: float, lon: float) -> str:
        return f"{round(lat, PadQueue._ROUND)},{round(lon, PadQueue._ROUND)}"

    def earliest_available(self, lat: float, lon: float,
                           not_before: float) -> float:
        """
        Return the earliest time >= not_before at which the pad is free.
        Accounts for PAD_CLEARANCE_TIME between consecutive users.
        """
        key = self._key(lat, lon)
        slots = self._slots.get(key, [])
        candidate = not_before
        for _, w_start, w_end in slots:
            # If candidate overlaps this slot, push past it
            if candidate < w_end + config.PAD_CLEARANCE_TIME:
                if w_end + config.PAD_CLEARANCE_TIME > candidate:
                    candidate = w_end + config.PAD_CLEARANCE_TIME
        return candidate

    def reserve(self, drone_id: str, lat: float, lon: float,
                window_start: float, window_end: float) -> None:
        key = self._key(lat, lon)
        if key not in self._slots:
            self._slots[key] = []
        self._slots[key].append((drone_id, window_start, window_end))
        self._slots[key].sort(key=lambda x: x[1])

    def release(self, drone_id: str) -> None:
        """Remove all slots belonging to drone_id (called on mission cleanup)."""
        for key in self._slots:
            self._slots[key] = [
                s for s in self._slots[key] if s[0] != drone_id
            ]


# Singleton pad queue (shared across requests)
pad_queue = PadQueue()


# ══════════════════════════════════════════════════════════════════════════════
# CONTINUOUS CPA  (Closest Point of Approach)
# ══════════════════════════════════════════════════════════════════════════════

def _segment_cpa(
    a0: Vec3, a1: Vec3, ta0: float, ta1: float,   # drone A segment
    b0: Vec3, b1: Vec3, tb0: float, tb1: float,   # drone B segment
) -> Tuple[float, float, float, float]:
    """
    Compute the minimum 3-D distance between two moving points over their
    shared time interval, using analytic closest-point-of-approach.

    Both drones move linearly within their segment:
        A(t) = a0 + (t - ta0) / (ta1 - ta0) * (a1 - a0)
        B(t) = b0 + (t - tb0) / (tb1 - tb0) * (b1 - b0)

    The relative velocity is constant, so ||A(t) - B(t)||^2 is a quadratic
    in t.  We minimise analytically and clamp to the shared time window.

    Returns (min_3d_dist, min_horiz_dist, min_vert_dist, cpa_time).
    """
    t_lo = max(ta0, tb0)
    t_hi = min(ta1, tb1)
    if t_lo >= t_hi:
        return float('inf'), float('inf'), float('inf'), t_lo

    dt_a = ta1 - ta0 if ta1 != ta0 else 1e-9
    dt_b = tb1 - tb0 if tb1 != tb0 else 1e-9

    # Velocity vectors (metres/second in local frame)
    va = ((a1[0]-a0[0])/dt_a, (a1[1]-a0[1])/dt_a, (a1[2]-a0[2])/dt_a)
    vb = ((b1[0]-b0[0])/dt_b, (b1[1]-b0[1])/dt_b, (b1[2]-b0[2])/dt_b)

    # Relative position at t=t_lo
    pa = (a0[0] + va[0]*(t_lo-ta0),
          a0[1] + va[1]*(t_lo-ta0),
          a0[2] + va[2]*(t_lo-ta0))
    pb = (b0[0] + vb[0]*(t_lo-tb0),
          b0[1] + vb[1]*(t_lo-tb0),
          b0[2] + vb[2]*(t_lo-tb0))

    r0  = (pa[0]-pb[0], pa[1]-pb[1], pa[2]-pb[2])
    vr  = (va[0]-vb[0], va[1]-vb[1], va[2]-vb[2])

    # d²(t) = |r0 + vr*(t-t_lo)|² = a*s² + b*s + c  where s = t-t_lo
    a_coef = vr[0]**2 + vr[1]**2 + vr[2]**2
    b_coef = 2*(r0[0]*vr[0] + r0[1]*vr[1] + r0[2]*vr[2])

    duration = t_hi - t_lo

    if a_coef < 1e-12:
        # Parallel / stationary relative motion — distance constant
        t_cpa = t_lo
    else:
        s_min = -b_coef / (2 * a_coef)
        s_min = max(0.0, min(s_min, duration))
        t_cpa = t_lo + s_min

    # Positions at t_cpa — relative vector only
    s = t_cpa - t_lo
    rel = (r0[0]+vr[0]*s, r0[1]+vr[1]*s, r0[2]+vr[2]*s)

    d3   = math.sqrt(rel[0]**2 + rel[1]**2 + rel[2]**2)
    d2   = math.sqrt(rel[0]**2 + rel[1]**2)
    dv   = abs(rel[2])
    return d3, d2, dv, t_cpa


def _effective_radius(wp_a: Waypoint4D, wp_b: Waypoint4D) -> float:
    """
    Effective horizontal protection radius combining both drones' uncertainty.
    R_eff = R_BASE + speed_A * sigma_A + speed_B * sigma_B
    """
    return (config.CPA_R_BASE
            + wp_a.speed * wp_a.sigma_t
            + wp_b.speed * wp_b.sigma_t)


def _wp_is_terminal(wp: Waypoint4D) -> bool:
    return wp.phase in (FlightPhase.TERMINAL, FlightPhase.GROUND)


# ══════════════════════════════════════════════════════════════════════════════
# TRAJECTORY SHIFT  (uniform takeoff delay)
# ══════════════════════════════════════════════════════════════════════════════

def apply_delay(traj: Trajectory4D, extra_delay: float) -> Trajectory4D:
    """
    Shift every ETA in the trajectory by extra_delay seconds.
    Keeps base_start_time and accumulates takeoff_delay for bookkeeping.
    """
    new_wps = []
    for wp in traj.waypoints:
        new_wps.append(Waypoint4D(
            position=wp.position,
            eta=wp.eta + extra_delay,
            speed=wp.speed,
            heading=wp.heading,
            phase=wp.phase,
            sigma_t=wp.sigma_t,
            anchor_bindings=wp.anchor_bindings,
        ))
    return Trajectory4D(
        waypoints=new_wps,
        total_distance=traj.total_distance,
        total_time=traj.total_time,
        estimated_battery_usage=traj.estimated_battery_usage,
        takeoff_delay=traj.takeoff_delay + extra_delay,
        base_start_time=traj.base_start_time,
        stratum=traj.stratum,
        direction=traj.direction,
    )



def reassign_stratum(traj: Trajectory4D,
                     new_stratum: int,
                     pickup_pos: Position,
                     delivery_pos: Position) -> Trajectory4D:
    """
    Return a new Trajectory4D with all CRUISE waypoints shifted to new_stratum,
    and CLIMB/DESCENT waypoints linearly interpolated to match.

    BUG FIX: anchor_bindings are now RE-COMPUTED for every waypoint at its
    new altitude instead of being blindly copied from the old trajectory.

    When reassign_stratum() copied the old bindings, two problems occurred:
      1. expected_dist_m was computed at the old altitude — the WLS residuals
         (measured − expected) had a large systematic bias, corrupting the
         position-correction vector.
      2. The node set was optimised for the old altitude geometry.  After a
         30 m stratum change some of those nodes fall outside ANCHOR_MAX_RANGE,
         while better-geometry nodes that are now in range go unused.

    Recomputing via compute_anchor_bindings() costs one GDOP pass per waypoint
    but runs entirely in the planning thread and is negligible compared to the
    OSRM / CPA work that precedes it.
    """
    old_stratum = traj.stratum
    if old_stratum == 0:
        old_stratum = new_stratum

    cruise_indices = [i for i, wp in enumerate(traj.waypoints)
                      if wp.phase == FlightPhase.CRUISE]
    first_cruise = cruise_indices[0]  if cruise_indices else len(traj.waypoints) // 2
    last_cruise  = cruise_indices[-1] if cruise_indices else len(traj.waypoints) // 2

    new_wps = []
    for i, wp in enumerate(traj.waypoints):
        pos = wp.position

        if wp.phase == FlightPhase.CRUISE:
            new_alt = float(new_stratum)

        elif wp.phase == FlightPhase.CLIMB:
            ratio   = pos.altitude / old_stratum if old_stratum else 0
            new_alt = ratio * new_stratum

        elif wp.phase == FlightPhase.DESCENT:
            ratio   = pos.altitude / old_stratum if old_stratum else 0
            new_alt = ratio * new_stratum

        elif wp.phase == FlightPhase.TERMINAL:
            ratio   = pos.altitude / old_stratum if old_stratum else 0
            new_alt = ratio * new_stratum

        else:
            new_alt = pos.altitude   # GROUND stays at 0

        new_alt = max(0.0, min(new_alt, 130.0))
        new_pos = Position(latitude=pos.latitude,
                           longitude=pos.longitude,
                           altitude=new_alt)

        # ── FIX: recompute anchor bindings for the new altitude ───────────────
        # The old bindings had expected_dist_m calibrated to the previous
        # stratum altitude.  Using them at the new altitude produces large
        # systematic WLS residuals and corrupts the position correction.
        wp_xyz = anchor_registry._to_xyz(pos.latitude, pos.longitude,
                                         min(new_alt, 130.0))
        new_bindings, _, _ = anchor_registry.compute_anchor_bindings(wp_xyz)
        # ─────────────────────────────────────────────────────────────────────

        new_wps.append(Waypoint4D(
            position=new_pos,
            eta=wp.eta,
            speed=wp.speed,
            heading=wp.heading,
            phase=wp.phase,
            sigma_t=wp.sigma_t,
            anchor_bindings=new_bindings,   # ← was: wp.anchor_bindings
        ))

    return Trajectory4D(
        waypoints=new_wps,
        total_distance=traj.total_distance,
        total_time=traj.total_time,
        estimated_battery_usage=traj.estimated_battery_usage,
        takeoff_delay=traj.takeoff_delay,
        base_start_time=traj.base_start_time,
        stratum=new_stratum,
        direction=traj.direction,
    )

# ══════════════════════════════════════════════════════════════════════════════
# CONFLICT CHECKER
# ══════════════════════════════════════════════════════════════════════════════

class ConflictInfo:
    """Lightweight conflict record used inside the resolver."""
    __slots__ = ('drone_id', 'cpa_dist_h', 'cpa_dist_v',
                 'cpa_time', 'r_eff', 'severity')

    def __init__(self, drone_id: str, cpa_dist_h: float, cpa_dist_v: float,
                 cpa_time: float, r_eff: float) -> None:
        self.drone_id   = drone_id
        self.cpa_dist_h = cpa_dist_h
        self.cpa_dist_v = cpa_dist_v
        self.cpa_time   = cpa_time
        self.r_eff      = r_eff
        self.severity   = ('critical' if cpa_dist_h < r_eff * 0.5 else
                           'warning'  if cpa_dist_h < r_eff * 0.75 else 'minor')


def check_trajectory_vs_all(
    candidate: Trajectory4D,
    existing:  Dict[str, Trajectory4D],
) -> List[ConflictInfo]:
    """
    Check candidate trajectory against every approved trajectory.
    Returns list of ConflictInfo (empty = clear).

    Skips segment pairs where BOTH endpoints are TERMINAL / GROUND phase —
    those are handled by the pad queue.
    """
    conflicts: List[ConflictInfo] = []

    for drone_id, traj in existing.items():
        worst_h   = float('inf')
        worst_v   = float('inf')
        worst_t   = 0.0
        worst_ref = 0.0

        wps_a = candidate.waypoints
        wps_b = traj.waypoints

        for i in range(len(wps_a) - 1):
            wp_a0, wp_a1 = wps_a[i], wps_a[i+1]
            # Skip if both endpoints are terminal/ground
            if _wp_is_terminal(wp_a0) and _wp_is_terminal(wp_a1):
                continue

            for j in range(len(wps_b) - 1):
                wp_b0, wp_b1 = wps_b[j], wps_b[j+1]
                if _wp_is_terminal(wp_b0) and _wp_is_terminal(wp_b1):
                    continue

                # Time window overlap?
                t_lo = max(wp_a0.eta, wp_b0.eta)
                t_hi = min(wp_a1.eta, wp_b1.eta)
                if t_lo >= t_hi:
                    continue

                # Convert to metric Cartesian
                a0m = _to_metres(wp_a0.position.latitude,
                                 wp_a0.position.longitude,
                                 wp_a0.position.altitude)
                a1m = _to_metres(wp_a1.position.latitude,
                                 wp_a1.position.longitude,
                                 wp_a1.position.altitude)
                b0m = _to_metres(wp_b0.position.latitude,
                                 wp_b0.position.longitude,
                                 wp_b0.position.altitude)
                b1m = _to_metres(wp_b1.position.latitude,
                                 wp_b1.position.longitude,
                                 wp_b1.position.altitude)

                _, d_h, d_v, t_cpa = _segment_cpa(
                    a0m, a1m, wp_a0.eta, wp_a1.eta,
                    b0m, b1m, wp_b0.eta, wp_b1.eta,
                )

                # Effective radius uses sigma_t at the CPA waypoints
                r_eff = _effective_radius(wp_a0, wp_b0)

                if d_h < r_eff and d_v < config.CPA_V_SEP:
                    if d_h < worst_h:
                        worst_h   = d_h
                        worst_v   = d_v
                        worst_t   = t_cpa
                        worst_ref = r_eff

        if worst_h < float('inf'):
            conflicts.append(ConflictInfo(
                drone_id=drone_id,
                cpa_dist_h=worst_h,
                cpa_dist_v=worst_v,
                cpa_time=worst_t,
                r_eff=worst_ref,
            ))

    return conflicts


# ══════════════════════════════════════════════════════════════════════════════
# MAIN RESOLVER
# ══════════════════════════════════════════════════════════════════════════════

class ConflictResolver:
    """
    Preventive 4-D conflict resolution applied at planning time.

    Usage (called from main.py):
        resolved_traj, alert = resolver.resolve(
            candidate_traj, existing_flight_plans,
            pickup_pos, delivery_pos
        )

    Returns (Trajectory4D, ConflictAlert|None).
    If the airspace is saturated, raises AirspaceSaturatedError.
    """

    def resolve(
        self,
        candidate:     Trajectory4D,
        existing:      Dict[str, Trajectory4D],
        pickup_pos:    Position,
        delivery_pos:  Position,
    ) -> Tuple[Trajectory4D, Optional[ConflictAlert]]:
        """
        Run the full cascade resolution loop.
        Returns final trajectory and a ConflictAlert if resolution was needed.
        Raises AirspaceSaturatedError if unresolvable.
        """
        if not existing:
            return candidate, None

        planning_start = time.monotonic()
        alert: Optional[ConflictAlert] = None
        traj = candidate
        total_delay_applied = 0.0

        for full_pass in range(config.MAX_RESOLUTION_PASSES):

            # Reset per-pass state so previously-failed strata are retried
            # after the conflict landscape changes (e.g. a delay was applied).
            tried_strata: set = set()

            # ── Wall-clock timeout ────────────────────────────────────────────
            elapsed = time.monotonic() - planning_start
            if elapsed > config.RESOLUTION_TIMEOUT:
                raise AirspaceSaturatedError(
                    f"Planning timeout after {elapsed:.1f}s",
                    retry_after=self._earliest_free_slot(candidate, existing),
                    conflicts=len(check_trajectory_vs_all(traj, existing)),
                )

            conflicts = check_trajectory_vs_all(traj, existing)

            if not conflicts:
                break   # Clean — done

            # Sort by earliest CPA time (fix the nearest conflict first)
            conflicts.sort(key=lambda c: c.cpa_time)
            worst = conflicts[0]

            print(f"[Resolver] Pass {full_pass+1}: "
                  f"{len(conflicts)} conflict(s), worst with drone "
                  f"{worst.drone_id} at t={worst.cpa_time:.1f}s "
                  f"d_h={worst.cpa_dist_h:.1f}m (need {worst.r_eff:.1f}m)")

            resolved_this_pass = False

            # ── Resolution 1: Try a different altitude stratum ────────────────
            strata_by_distance = sorted(
                [s for s in config.ALL_STRATA_ORDERED
                 if s != traj.stratum and s not in tried_strata],
                key=lambda s: abs(s - traj.stratum)
            )

            for alt_s in strata_by_distance:
                tried_strata.add(alt_s)
                candidate_alt = reassign_stratum(traj, alt_s,
                                                 pickup_pos, delivery_pos)
                if not check_trajectory_vs_all(candidate_alt, existing):
                    print(f"[Resolver] ✓ Resolution 1: stratum {traj.stratum}m "
                          f"-> {alt_s}m clears all conflicts")
                    traj = candidate_alt
                    alert = self._make_alert(worst, traj, "altitude_change")
                    resolved_this_pass = True
                    break

            if resolved_this_pass:
                continue   # Re-validate on next full pass

            # ── Resolution 2: Uniform takeoff delay ──────────────────────────
            # Binary-search the minimum delay in [lo, MAX_TAKEOFF_DELAY]
            # that makes the trajectory clear of all existing traffic.
            combined_sigma = (
                traj.waypoints[0].sigma_t
                + existing[worst.drone_id].waypoints[0].sigma_t
            )
            delay_lo  = combined_sigma + config.TEMPORAL_SAFETY_MARGIN
            delay_hi  = config.MAX_TAKEOFF_DELAY - total_delay_applied

            if delay_lo > delay_hi:
                # Already at max delay budget
                retry = self._earliest_free_slot(candidate, existing)
                raise AirspaceSaturatedError(
                    "All strata blocked and max delay budget exhausted",
                    retry_after=retry,
                    conflicts=len(conflicts),
                )

            best_delay = self._binary_search_delay(
                traj, existing, delay_lo, delay_hi
            )

            if best_delay is not None:
                traj = apply_delay(traj, best_delay)
                total_delay_applied += best_delay
                print(f"[Resolver] ✓ Resolution 2: delay +{best_delay:.1f}s "
                      f"(total {total_delay_applied:.1f}s)")
                alert = self._make_alert(worst, traj, "speed_adjustment")
                resolved_this_pass = True
                continue

            # Both resolutions exhausted
            retry = self._earliest_free_slot(candidate, existing)
            raise AirspaceSaturatedError(
                "Could not find conflict-free trajectory",
                retry_after=retry,
                conflicts=len(conflicts),
            )

        else:
            # Exceeded MAX_RESOLUTION_PASSES
            retry = self._earliest_free_slot(candidate, existing)
            raise AirspaceSaturatedError(
                f"Exceeded {config.MAX_RESOLUTION_PASSES} resolution passes",
                retry_after=retry,
                conflicts=len(check_trajectory_vs_all(traj, existing)),
            )

        return traj, alert

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _binary_search_delay(
        self,
        traj:     Trajectory4D,
        existing: Dict[str, Trajectory4D],
        lo:       float,
        hi:       float,
        n_iter:   int = 8,
    ) -> Optional[float]:
        """
        Binary-search for the minimum delay in [lo, hi] that clears all conflicts.
        Returns the delay in seconds, or None if even hi doesn't clear them.
        """
        # First check if hi works at all
        if check_trajectory_vs_all(apply_delay(traj, hi), existing):
            return None   # Even max delay doesn't help

        # Fast-path: if the minimum delay already clears everything, use it
        if not check_trajectory_vs_all(apply_delay(traj, lo), existing):
            return lo

        for _ in range(n_iter):
            mid = (lo + hi) / 2
            if check_trajectory_vs_all(apply_delay(traj, mid), existing):
                lo = mid   # mid still has conflicts — need more delay
            else:
                hi = mid   # mid is clean — try less

        return hi   # smallest delay that was consistently clean

    def _earliest_free_slot(
        self,
        candidate: Trajectory4D,
        existing:  Dict[str, Trajectory4D],
    ) -> float:
        """
        Estimate the earliest future time when the airspace might clear
        (the end of the last in-flight trajectory).
        """
        if not existing:
            return time.time()
        latest_end = max(t.waypoints[-1].eta for t in existing.values())
        return latest_end + config.PAD_CLEARANCE_TIME

    def _make_alert(
        self,
        conflict: ConflictInfo,
        resolved_traj: Trajectory4D,
        method: str,
    ) -> ConflictAlert:
        # Find closest waypoint to conflict time for position
        wps = resolved_traj.waypoints
        closest = min(wps, key=lambda w: abs(w.eta - conflict.cpa_time))
        return ConflictAlert(
            conflict_id=str(uuid.uuid4()),
            drone_1_id="new_drone",
            drone_2_id=conflict.drone_id,
            conflict_position=closest.position,
            conflict_time=conflict.cpa_time,
            severity=conflict.severity,
            resolution_action=method,
        )

    # ── Legacy compatibility (called from old main.py code) ───────────────────
    conflict_detector = None  # replaced by check_trajectory_vs_all

    def check_trajectory_conflict(self, id1, traj1, id2, traj2):
        """Legacy shim — returns ConflictAlert-compatible object or None."""
        t1_4d = _legacy_to_4d(traj1)
        t2_4d = _legacy_to_4d(traj2)
        infos = check_trajectory_vs_all(t1_4d, {id2: t2_4d})
        if not infos:
            return None
        info = infos[0]
        wps = t1_4d.waypoints
        closest = min(wps, key=lambda w: abs(w.eta - info.cpa_time))
        return ConflictAlert(
            conflict_id=str(uuid.uuid4()),
            drone_1_id=id1,
            drone_2_id=id2,
            conflict_position=closest.position,
            conflict_time=info.cpa_time,
            severity=info.severity,
            resolution_action=None,
        )


def _legacy_to_4d(traj) -> Trajectory4D:
    """Wrap a legacy Trajectory in a minimal Trajectory4D for CPA checks."""
    from models import Waypoint4D, FlightPhase
    wps4d = [
        Waypoint4D(
            position=wp.position,
            eta=wp.eta,
            speed=wp.speed,
            heading=wp.heading,
            phase=FlightPhase.CRUISE,
            sigma_t=config.CPA_SIGMA_0,
        )
        for wp in traj.waypoints
    ]
    return Trajectory4D(
        waypoints=wps4d,
        total_distance=traj.total_distance,
        total_time=traj.total_time,
        estimated_battery_usage=traj.estimated_battery_usage,
    )


# ══════════════════════════════════════════════════════════════════════════════
# ERRORS
# ══════════════════════════════════════════════════════════════════════════════

class AirspaceSaturatedError(Exception):
    def __init__(self, reason: str, retry_after: float, conflicts: int) -> None:
        super().__init__(reason)
        self.retry_after = retry_after
        self.conflicts   = conflicts


# ── Legacy class kept for import compatibility ────────────────────────────────
class ConflictDetector:
    """Legacy stub — CPA logic is now in check_trajectory_vs_all()."""
    def __init__(self):
        self.active_conflicts = []

    def check_trajectory_conflict(self, id1, traj1, id2, traj2):
        resolver = ConflictResolver()
        return resolver.check_trajectory_conflict(id1, traj1, id2, traj2)
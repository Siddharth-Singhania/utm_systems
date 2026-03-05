"""
ConflictResolver — Planning-time 4-D conflict detection and resolution

Overview
--------
Every newly planned trajectory is checked against all currently active
trajectories using continuous Closest-Point-of-Approach (CPA) math before the
drone is committed to the plan.  If a conflict is found, the resolver applies
two resolution layers in order:

  Layer 1  — Altitude (Stratum) Reassignment
    Try every stratum in config.ALL_STRATA_ORDERED for the lower-priority
    drone.  Accept the first stratum that is conflict-free.

  Layer 2  — Path vs. Speed Trade-off  (runs only if Layer 1 exhausted)
    Option A: Keep the primary path; binary-search for the minimum cruise
              speed V_reduced ∈ [DRONE_MIN_SPEED, V_cruise] that resolves all
              conflicts.  Compute final ETA including kinematic turn penalties
              at V_reduced (corners are slower at reduced cruise speed).

    Option B: Test each Yen's alternative trajectory (paths 2-5, pre-built by
              plan_trajectory at full speed V_cruise) against all active
              drones.  Find the fastest conflict-free alternative.

    Selection: compare final ETA of Option A vs. Option B.  Accept whichever
               drone lands first.  If neither option succeeds, fall back to a
               binary-searched takeoff delay on the primary trajectory (the
               original behaviour).

CPA Model (Analytic Quadratic)
-------------------------------
Two drones A and B are modelled as moving linearly between consecutive
waypoints.  For overlapping time windows [t_lo, t_hi]:

    d(s) = D₀ + s · Dv          s ∈ [0, Δt],  Δt = t_hi − t_lo
    D₀   = Pa(t_lo) − Pb(t_lo)  (initial separation vector)
    Dv   = Va − Vb               (relative velocity vector in lat/lon/alt)

    |d(s)|² = |Dv|² s² + 2(D₀·Dv) s + |D₀|²

    t_cpa = −(D₀·Dv) / |Dv|²   → clamp to [0, Δt]
    d_cpa = |d(t_cpa)|

Conflict when:
    d_cpa_horizontal  <  R_eff   (metres, haversine-equivalent)
    d_cpa_vertical    <  V_sep   (metres, absolute altitude difference)

where  R_eff = DRONE_PROTECTION_RADIUS + CPA_SIGMA_SCALE × (σ_t_A + σ_t_B)
expands the protected cylinder proportionally to the combined timing
uncertainty of the two waypoints.

Priority Rule
-------------
Lower drone_id (lexicographic / insertion order) = higher priority.
Higher-priority drones are never modified; only the lower-priority drone's
trajectory is adjusted.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

from models import Position, Waypoint4D, Trajectory4D, FlightPhase
import config
import pathfinding


# ══════════════════════════════════════════════════════════════════════════════
# CONSTANTS
# ══════════════════════════════════════════════════════════════════════════════

# How many seconds of temporal buffer to verify that a proposed resolution
# truly separates the two drones — checked on the full trajectory.
_CPA_CHECK_BUFFER: float = 0.0

# How many binary-search iterations for speed reduction (Option A).
_SPEED_BISECT_ITERS: int = 20

# How many binary-search iterations for takeoff-delay fallback.
_DELAY_BISECT_ITERS: int = 24

# Scale factor for sigma_t expansion of the protection radius.
# R_eff = PROTECTION_RADIUS + CPA_SIGMA_SCALE × (σ_t_a + σ_t_b)
_CPA_SIGMA_SCALE: float = getattr(config, 'CPA_SIGMA_SCALE', 1.0)

# Resolution labels returned in the info string.
_RES_STRATUM   = 'LAYER1_STRATUM'
_RES_SPEED     = 'LAYER2_SPEED'
_RES_ALT_PATH  = 'LAYER2_ALT_PATH'
_RES_DELAY     = 'FALLBACK_DELAY'
_RES_NONE      = 'UNRESOLVED'


# ══════════════════════════════════════════════════════════════════════════════
# LOW-LEVEL CPA GEOMETRY
# ══════════════════════════════════════════════════════════════════════════════

def _lat_lon_to_xy_local(lat: float, lon: float,
                          ref_lat: float, ref_lon: float) -> Tuple[float, float]:
    """
    Convert (lat, lon) to approximate Cartesian metres relative to a
    reference point.  Accurate to ~0.1 % within 10 km — sufficient for CPA.
    """
    R = 6_371_000.0
    x = math.radians(lon - ref_lon) * R * math.cos(math.radians(ref_lat))
    y = math.radians(lat - ref_lat) * R
    return x, y


def _wp_xyz(wp: Waypoint4D,
             ref_lat: float, ref_lon: float) -> Tuple[float, float, float]:
    x, y = _lat_lon_to_xy_local(
        wp.position.latitude, wp.position.longitude, ref_lat, ref_lon
    )
    return x, y, wp.position.altitude


def _cpa_segment_pair(
    ax0: float, ay0: float, az0: float,
    ax1: float, ay1: float, az1: float,
    bx0: float, by0: float, bz0: float,
    bx1: float, by1: float, bz1: float,
    ta0: float, ta1: float,
    tb0: float, tb1: float,
    r_eff: float,
    v_sep: float,
) -> bool:
    """
    Check whether two linear trajectory segments conflict.

    Each drone moves at constant velocity between two 3-D positions over its
    own time interval.  The function finds the time of minimum separation
    within the overlapping portion of those intervals.

    Returns True if a conflict (violation of protected airspace) is found.
    """
    t_lo = max(ta0, tb0)
    t_hi = min(ta1, tb1)
    if t_hi <= t_lo:
        return False               # no temporal overlap

    # Fraction of each drone's segment occupied by the overlap window
    da = ta1 - ta0
    db = tb1 - tb0
    if da < 1e-9 or db < 1e-9:
        return False

    fa0 = (t_lo - ta0) / da
    fa1 = (t_hi - ta0) / da
    fb0 = (t_lo - tb0) / db
    fb1 = (t_hi - tb0) / db

    # 3-D positions at the start of the overlap window
    Pa = (ax0 + fa0 * (ax1 - ax0),
          ay0 + fa0 * (ay1 - ay0),
          az0 + fa0 * (az1 - az0))
    Qa = (ax0 + fa1 * (ax1 - ax0),
          ay0 + fa1 * (ay1 - ay0),
          az0 + fa1 * (az1 - az0))

    Pb = (bx0 + fb0 * (bx1 - bx0),
          by0 + fb0 * (by1 - by0),
          bz0 + fb0 * (bz1 - bz0))
    Qb = (bx0 + fb1 * (bx1 - bx0),
          by0 + fb1 * (by1 - by0),
          bz0 + fb1 * (bz1 - bz0))

    dt = t_hi - t_lo

    # Velocity vectors during the overlap
    Vax = (Qa[0] - Pa[0]) / dt;  Vay = (Qa[1] - Pa[1]) / dt
    Vaz = (Qa[2] - Pa[2]) / dt
    Vbx = (Qb[0] - Pb[0]) / dt;  Vby = (Qb[1] - Pb[1]) / dt
    Vbz = (Qb[2] - Pb[2]) / dt

    # Relative motion: d(s) = D0 + s * Dv
    D0x = Pa[0] - Pb[0];  D0y = Pa[1] - Pb[1];  D0z = Pa[2] - Pb[2]
    Dvx = Vax - Vbx;      Dvy = Vay - Vby;      Dvz = Vaz - Vbz

    Dv2 = Dvx*Dvx + Dvy*Dvy + Dvz*Dvz
    if Dv2 < 1e-12:
        # Drones stationary relative to each other — check initial separation
        t_cpa_s = 0.0
    else:
        # t_cpa = −(D0 · Dv) / |Dv|²    (unclamped)
        dot = D0x*Dvx + D0y*Dvy + D0z*Dvz
        t_cpa_s = max(0.0, min(dt, -dot / Dv2))

    cx = D0x + t_cpa_s * Dvx
    cy = D0y + t_cpa_s * Dvy
    cz = D0z + t_cpa_s * Dvz

    d_h = math.sqrt(cx*cx + cy*cy)
    d_v = abs(cz)

    return d_h < r_eff and d_v < v_sep


# ══════════════════════════════════════════════════════════════════════════════
# TRAJECTORY-LEVEL CONFLICT CHECK
# ══════════════════════════════════════════════════════════════════════════════

def trajectories_conflict(
    traj_a: Trajectory4D,
    traj_b: Trajectory4D,
) -> bool:
    """
    Return True if traj_a and traj_b ever violate the protected airspace
    cylinder (horizontal R_eff × vertical V_SEP) at any point during their
    overlapping flight time.

    Uses pairwise analytic CPA over all segment combinations.
    Complexity: O(|A| × |B|) — manageable at planning time (< 50 waypoints
    per trajectory).

    GROUND-phase waypoints are excluded (drones on the pad do not occupy
    airspace).
    """
    wps_a = [w for w in traj_a.waypoints if w.phase != FlightPhase.GROUND]
    wps_b = [w for w in traj_b.waypoints if w.phase != FlightPhase.GROUND]

    if not wps_a or not wps_b:
        return False

    # Reference point for local Cartesian conversion
    ref_lat = wps_a[0].position.latitude
    ref_lon = wps_a[0].position.longitude

    r_base = getattr(config, 'DRONE_PROTECTION_RADIUS', 50.0)
    v_sep  = getattr(config, 'DRONE_VERTICAL_SEPARATION', 10.0)

    # Pre-convert all waypoints to (x, y, z, eta, sigma_t) tuples
    def _convert(wps):
        out = []
        for w in wps:
            x, y = _lat_lon_to_xy_local(
                w.position.latitude, w.position.longitude, ref_lat, ref_lon
            )
            out.append((x, y, w.position.altitude, w.eta, w.sigma_t))
        return out

    pts_a = _convert(wps_a)
    pts_b = _convert(wps_b)

    for i in range(len(pts_a) - 1):
        ax0, ay0, az0, ta0, sa0 = pts_a[i]
        ax1, ay1, az1, ta1, sa1 = pts_a[i + 1]

        for j in range(len(pts_b) - 1):
            bx0, by0, bz0, tb0, sb0 = pts_b[j]
            bx1, by1, bz1, tb1, sb1 = pts_b[j + 1]

            # Effective protection radius expands with combined sigma_t
            avg_sigma = 0.5 * (sa0 + sa1 + sb0 + sb1) / 2.0
            r_eff = r_base + _CPA_SIGMA_SCALE * avg_sigma

            if _cpa_segment_pair(
                ax0, ay0, az0, ax1, ay1, az1,
                bx0, by0, bz0, bx1, by1, bz1,
                ta0, ta1, tb0, tb1,
                r_eff, v_sep,
            ):
                return True

    return False


def has_any_conflict(candidate: Trajectory4D,
                     active_trajs: Dict[str, Trajectory4D],
                     exclude_id: Optional[str] = None) -> bool:
    """
    Return True if `candidate` conflicts with ANY trajectory in `active_trajs`.
    Skips `exclude_id` (typically the drone being replanned).
    """
    for drone_id, traj in active_trajs.items():
        if drone_id == exclude_id:
            continue
        if trajectories_conflict(candidate, traj):
            return True
    return False


# ══════════════════════════════════════════════════════════════════════════════
# TRAJECTORY HELPERS
# ══════════════════════════════════════════════════════════════════════════════

def _extract_latlon_path(traj: Trajectory4D) -> List[Tuple[float, float]]:
    """Extract (lat, lon) pairs from CRUISE + TERMINAL waypoints (not GROUND)."""
    return [
        (w.position.latitude, w.position.longitude)
        for w in traj.waypoints
        if w.phase not in (FlightPhase.GROUND,)
    ]


def _final_eta(traj: Trajectory4D) -> float:
    """Return the ETA of the last waypoint (touchdown)."""
    return traj.waypoints[-1].eta


def _shift_trajectory_eta(traj: Trajectory4D, delay: float) -> Trajectory4D:
    """
    Return a shallow copy of traj with every waypoint ETA shifted forward
    by `delay` seconds.  Used for takeoff-delay fallback.
    """
    from copy import deepcopy
    new_traj = deepcopy(traj)
    for wp in new_traj.waypoints:
        wp.eta += delay
    new_traj.base_start_time = traj.base_start_time + delay
    return new_traj


# ══════════════════════════════════════════════════════════════════════════════
# STRATUM REBUILDER
# ══════════════════════════════════════════════════════════════════════════════

def _rebuild_at_stratum(
    traj:        Trajectory4D,
    new_stratum: int,
    start_pos:   Position,
    goal_pos:    Position,
    latlon_path: List[Tuple[float, float]],
) -> Trajectory4D:
    """
    Rebuild the trajectory at a different altitude stratum.

    Recomputes anchor bindings from scratch (not a copy) so that
    expected_dist_m values are computed for the new altitude.  Copying them
    from the old trajectory would corrupt WLS residuals when the drone is
    handed to the GPS-denied positioning module.
    """
    return pathfinding.build_4d_trajectory(
        latlon_path,
        start_pos,
        goal_pos,
        new_stratum,
        traj.direction,
        traj.base_start_time,
    )


# ══════════════════════════════════════════════════════════════════════════════
# OPTION A — SPEED REDUCTION
# ══════════════════════════════════════════════════════════════════════════════

def _speed_reduced_trajectory(
    traj:          Trajectory4D,
    start_pos:     Position,
    goal_pos:      Position,
    latlon_path:   List[Tuple[float, float]],
    target_speed:  float,
) -> Trajectory4D:
    """
    Rebuild the trajectory with a lower cruise speed.

    Kinematic turn penalties are re-evaluated at the new speed:
        V_corner = target_speed × cos(Δθ / 2)   [clamped to DRONE_MIN_SPEED]
    so that corner-slow-down effects are realistic at the reduced cruise.

    This is used by Option A of the Path-vs-Speed resolver.
    """
    return pathfinding.build_4d_trajectory(
        latlon_path,
        start_pos,
        goal_pos,
        traj.stratum,
        traj.direction,
        traj.base_start_time,
        cruise_speed_override=target_speed,
    )


def _find_min_speed_no_conflict(
    traj:         Trajectory4D,
    start_pos:    Position,
    goal_pos:     Position,
    latlon_path:  List[Tuple[float, float]],
    active_trajs: Dict[str, Trajectory4D],
    exclude_id:   Optional[str],
) -> Optional[Tuple[float, Trajectory4D]]:
    """
    Binary-search for the minimum cruise speed at which the primary path is
    conflict-free.

    Search range: [DRONE_MIN_SPEED, current stratum cruise speed].
    Turn penalties are included in both the rebuilt trajectory (for CPA
    checking) and the ETA estimate, ensuring a fair comparison with Option B.

    Returns
    -------
    (speed_found, rebuilt_trajectory) if a speed was found, else None.

    The returned speed is the slowest that just barely clears all conflicts,
    so the returned trajectory's ETA is the best achievable on this path.
    """
    v_min    = config.DRONE_MIN_SPEED
    v_max    = config.DRONE_CRUISE_SPEED
    v_lo, v_hi = v_min, v_max

    # Fast check: even at minimum speed, do we conflict?
    min_speed_traj = _speed_reduced_trajectory(
        traj, start_pos, goal_pos, latlon_path, v_lo
    )
    if has_any_conflict(min_speed_traj, active_trajs, exclude_id):
        # Speed reduction cannot resolve the conflict on this path
        return None

    # Confirmed: somewhere between v_lo and v_hi there is a working speed.
    # Binary search for the *fastest* speed that is still conflict-free
    # (we want earliest arrival → highest speed that works).
    #
    # Invariant: v_lo always works, v_hi may or may not.
    best_speed = v_lo
    best_traj  = min_speed_traj

    for _ in range(_SPEED_BISECT_ITERS):
        v_mid = (v_lo + v_hi) / 2.0
        if v_mid - v_lo < 0.05:   # 0.05 m/s resolution is sufficient
            break
        candidate = _speed_reduced_trajectory(
            traj, start_pos, goal_pos, latlon_path, v_mid
        )
        if has_any_conflict(candidate, active_trajs, exclude_id):
            v_hi = v_mid           # too fast — still conflicts
        else:
            v_lo      = v_mid      # faster and clear — keep as best
            best_speed = v_mid
            best_traj  = candidate

    print(f"[Resolver] Option A: speed reduced to "
          f"{best_speed:.1f} m/s, ETA={_final_eta(best_traj):.1f}s")
    return best_speed, best_traj


# ══════════════════════════════════════════════════════════════════════════════
# OPTION B — ALTERNATIVE PATH SELECTION
# ══════════════════════════════════════════════════════════════════════════════

def _find_fastest_clear_alternative(
    alternatives:  List[Trajectory4D],
    active_trajs:  Dict[str, Trajectory4D],
    exclude_id:    Optional[str],
) -> Optional[Trajectory4D]:
    """
    Scan the pre-built Yen's alternative trajectories (paths 2-5) in order
    of ascending travel time (they arrive sorted that way from plan_trajectory)
    and return the first one that is conflict-free.

    The alternatives are already built with full cruise speed V_cruise and
    include kinematic turn penalties in their ETAs (because they were built
    by build_4d_trajectory which uses _turn_speed() per CRUISE waypoint).

    Returns the earliest-arrival conflict-free alternative, or None.
    """
    for idx, alt in enumerate(alternatives):
        if not has_any_conflict(alt, active_trajs, exclude_id):
            print(f"[Resolver] Option B: alternative path {idx+1} is clear, "
                  f"ETA={_final_eta(alt):.1f}s")
            return alt

    print("[Resolver] Option B: all alternatives still conflict")
    return None


# ══════════════════════════════════════════════════════════════════════════════
# FALLBACK — TAKEOFF DELAY
# ══════════════════════════════════════════════════════════════════════════════

def _find_min_delay_no_conflict(
    traj:         Trajectory4D,
    active_trajs: Dict[str, Trajectory4D],
    exclude_id:   Optional[str],
) -> Optional[Trajectory4D]:
    """
    Binary-search the smallest takeoff delay that makes the primary trajectory
    conflict-free.

    This is the original fallback behaviour, applied only when both Layer 1
    and Layer 2 fail to resolve the conflict on any path or speed.
    Maximum search window: 10 minutes.
    """
    d_lo, d_hi = 0.0, 600.0

    # Verify that some delay actually works
    shifted = _shift_trajectory_eta(traj, d_hi)
    if has_any_conflict(shifted, active_trajs, exclude_id):
        return None    # even 10-minute delay doesn't help

    for _ in range(_DELAY_BISECT_ITERS):
        d_mid = (d_lo + d_hi) / 2.0
        if d_hi - d_lo < 0.5:
            break
        if has_any_conflict(_shift_trajectory_eta(traj, d_mid),
                            active_trajs, exclude_id):
            d_lo = d_mid
        else:
            d_hi = d_mid

    result = _shift_trajectory_eta(traj, d_hi)
    print(f"[Resolver] Fallback: takeoff delay {d_hi:.1f}s, "
          f"ETA={_final_eta(result):.1f}s")
    return result


# ══════════════════════════════════════════════════════════════════════════════
# MAIN RESOLVER
# ══════════════════════════════════════════════════════════════════════════════

def resolve_conflict(
    drone_id:     str,
    primary:      Trajectory4D,
    alternatives: List[Trajectory4D],
    active_trajs: Dict[str, Trajectory4D],
    start_pos:    Position,
    goal_pos:     Position,
    latlon_path:  List[Tuple[float, float]],
) -> Tuple[Trajectory4D, str]:
    """
    Resolve all conflicts for `drone_id`'s proposed primary trajectory.

    Parameters
    ----------
    drone_id     : ID of the drone being planned (excluded from self-conflict
                   checks; lower ID = higher priority).
    primary      : The freshly planned primary trajectory.
    alternatives : Yen's alternative trajectories (paths 2-5), pre-built at
                   full speed, sorted by travel time ascending.
    active_trajs : All currently active drone trajectories (including others
                   whose IDs are lexicographically lower than drone_id — those
                   are higher-priority and immovable).
    start_pos    : Original pickup position (needed for stratum + speed rebuild).
    goal_pos     : Original delivery position.
    latlon_path  : 2-D road/visibility waypoints of the primary trajectory
                   (needed for rebuild without re-running pathfinding).

    Returns
    -------
    (resolved_trajectory, resolution_method_label)

    Resolution method labels: see module-level _RES_* constants.
    """
    # Fast exit — no conflict with anyone
    if not has_any_conflict(primary, active_trajs, exclude_id=drone_id):
        return primary, 'NO_CONFLICT'

    print(f"[Resolver] Conflict detected for {drone_id} — starting resolution")

    # ── LAYER 1: Stratum reassignment ─────────────────────────────────────────
    all_strata = getattr(config, 'ALL_STRATA_ORDERED',
                         [60, 70, 80, 90, 100, 110, 120])

    for new_stratum in all_strata:
        if new_stratum == primary.stratum:
            continue                    # already tried the primary stratum

        candidate = _rebuild_at_stratum(
            primary, new_stratum, start_pos, goal_pos, latlon_path
        )
        if not has_any_conflict(candidate, active_trajs, exclude_id=drone_id):
            print(f"[Resolver] Layer 1 ✓ stratum={new_stratum}m, "
                  f"ETA={_final_eta(candidate):.1f}s")
            return candidate, f'{_RES_STRATUM}_{new_stratum}m'

    print("[Resolver] Layer 1 exhausted — all strata conflict")

    # ── LAYER 2: Path vs. Speed trade-off ─────────────────────────────────────

    # --- Option A: speed reduction on the primary path ----------------------
    option_a_result = _find_min_speed_no_conflict(
        primary, start_pos, goal_pos, latlon_path,
        active_trajs, exclude_id=drone_id
    )

    # --- Option B: fastest conflict-free Yen's alternative ------------------
    # Skip path 0 (same as primary), use paths 1 .. K-1
    option_b_result = _find_fastest_clear_alternative(
        alternatives[1:],          # paths 2-5 (index 1 onward)
        active_trajs,
        exclude_id=drone_id,
    )

    # --- Select: whichever ETA is earlier -----------------------------------
    if option_a_result is not None and option_b_result is not None:
        _, traj_a = option_a_result
        eta_a = _final_eta(traj_a)
        eta_b = _final_eta(option_b_result)

        if eta_a <= eta_b:
            print(f"[Resolver] Layer 2 ✓ Option A wins "
                  f"(ETA_A={eta_a:.1f}s < ETA_B={eta_b:.1f}s)")
            return traj_a, _RES_SPEED
        else:
            print(f"[Resolver] Layer 2 ✓ Option B wins "
                  f"(ETA_B={eta_b:.1f}s < ETA_A={eta_a:.1f}s)")
            return option_b_result, _RES_ALT_PATH

    elif option_a_result is not None:
        _, traj_a = option_a_result
        print(f"[Resolver] Layer 2 ✓ Option A only "
              f"(ETA={_final_eta(traj_a):.1f}s)")
        return traj_a, _RES_SPEED

    elif option_b_result is not None:
        print(f"[Resolver] Layer 2 ✓ Option B only "
              f"(ETA={_final_eta(option_b_result):.1f}s)")
        return option_b_result, _RES_ALT_PATH

    print("[Resolver] Layer 2 exhausted — falling back to takeoff delay")

    # ── FALLBACK: Takeoff delay on primary trajectory ─────────────────────────
    delayed = _find_min_delay_no_conflict(primary, active_trajs, drone_id)
    if delayed is not None:
        return delayed, _RES_DELAY

    # ── UNRESOLVED ─────────────────────────────────────────────────────────────
    print(f"[Resolver] ⚠ Could not resolve conflict for {drone_id}")
    return primary, _RES_NONE


# ══════════════════════════════════════════════════════════════════════════════
# BACKWARD-COMPAT LAYER  (consumed by main.py — do not remove)
# ══════════════════════════════════════════════════════════════════════════════

# Public alias so main.py's `conflict_detection.apply_delay(traj, d)` still works.
apply_delay = _shift_trajectory_eta


class AirspaceSaturatedError(Exception):
    """
    Raised when no resolution strategy can clear a conflict.
    Carries enough context for main.py to return a 503 with retry guidance.
    """
    def __init__(self, message: str,
                 retry_after: float = 30.0,
                 conflicts: Optional[List[str]] = None):
        super().__init__(message)
        self.retry_after = retry_after
        self.conflicts   = conflicts or []


# ── Pad Queue ─────────────────────────────────────────────────────────────────
# Minimal FIFO pad-slot manager.  Preserves the original PAD_QUEUE_ENABLED
# bypass behaviour: when False, every pad is treated as immediately available.

PAD_QUEUE_ENABLED: bool = getattr(config, 'PAD_QUEUE_ENABLED', False)


class _PadQueue:
    """
    Tracks time-window reservations for landing pads.

    Each pad is identified by a (lat, lon) pair rounded to 5 decimal places.
    Each reservation is keyed by drone_id and stores (start, end) in epoch
    seconds.  Pad slots are released explicitly by drone_id.
    """

    def __init__(self) -> None:
        # pad_key → list of (drone_id, start, end)
        self._slots: Dict[str, List[tuple]] = {}

    def _pad_key(self, lat: float, lon: float) -> str:
        return f"{round(lat, 5)},{round(lon, 5)}"

    def reserve(self, drone_id: str,
                lat: float, lon: float,
                start: float, end: float) -> None:
        key = self._pad_key(lat, lon)
        if key not in self._slots:
            self._slots[key] = []
        # Remove any existing reservation for this drone on this pad first
        self._slots[key] = [s for s in self._slots[key] if s[0] != drone_id]
        self._slots[key].append((drone_id, start, end))

    def release(self, drone_id: str) -> None:
        for key in list(self._slots):
            self._slots[key] = [s for s in self._slots[key] if s[0] != drone_id]

    def earliest_available(self, lat: float, lon: float,
                           not_before: float) -> float:
        """
        Return the earliest time >= not_before at which this pad is free.
        """
        key      = self._pad_key(lat, lon)
        slots    = self._slots.get(key, [])
        earliest = not_before

        # Walk forward through overlapping reservations
        changed = True
        while changed:
            changed = False
            for _, start, end in slots:
                if start < earliest + getattr(config, 'PAD_CLEARANCE_TIME', 30.0) \
                        and end > earliest:
                    earliest = end
                    changed  = True

        return earliest


pad_queue = _PadQueue()
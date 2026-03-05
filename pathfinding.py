"""
Pathfinding + 4-D Trajectory Builder

Public API
----------
get_direct_path(start, goal)          -> List[LatLon]     (straight line, for reroute viz)
get_k_fastest_paths(start, goal, k)   -> List[List[LatLon]] (K time-optimal paths)
plan_trajectory(start, goal,
                start_time,
                stratum, direction)   -> Optional[Tuple[Trajectory4D, List[Trajectory4D]]]
                                         Returns (primary_trajectory, alternative_trajectories)

Internal pipeline
-----------------
Level 1  OSRM direct road route        validated against no-fly zones
Level 2  OSRM blocked -> get_k_fastest_paths() bypass corners,
         snapped to nearest road, chained OSRM route
Level 3  Pure get_k_fastest_paths() visibility graph A*  (last resort)

After a 2-D road path is found, build_4d_trajectory() converts it into a
full Trajectory4D with:
  - GROUND waypoint at z=0 on the launch pad
  - TERMINAL/CLIMB synthetic waypoints (ascending to stratum)
  - CRUISE waypoints (OSRM road points at stratum altitude)
  - DESCENT/TERMINAL synthetic waypoints (descending to delivery pad)
  - GROUND waypoint at z=0 on the delivery pad
  - sigma_t accumulated per waypoint (ETA uncertainty cone)
  - phase tag on every waypoint
  - turn-adjusted speed per CRUISE waypoint (cosine model + accel penalty)

Time-Optimal Routing (Yen's K=5 + Time-A*)
-------------------------------------------
get_k_fastest_paths() generates up to K loopless candidate paths using
Yen's algorithm.  The inner A* uses TIME-based g-scores:

  g(edge) = dist / V_cruise  [+ kinematic turn penalty when |Δθ| > 20°]

  Kinematic turn penalty (deceleration + re-acceleration):
      V_corner = V_cruise × cos(Δθ / 2)
      τ        = 2 × (V_cruise − V_corner) / DRONE_ACCEL_LIMIT

  Admissible heuristic:
      h(n) = haversine(n, goal) / DRONE_MAX_SPEED

All K paths are returned sorted by total travel time (including turn
penalties).  The conflict resolver uses paths 2-5 as pre-built alternative
trajectories for the Path-vs-Speed trade-off in Layer 2.
"""

import math
import heapq
import requests
from typing import List, Optional, Tuple

from models import (
    Position, Waypoint, Waypoint4D, Trajectory, Trajectory4D, FlightPhase
)
import config
import geofencing
import anchors as anchor_registry

# ── OSRM ─────────────────────────────────────────────────────────────────────
OSRM_BASE    = "http://router.project-osrm.org/route/v1/driving"
OSRM_NEAREST = "http://router.project-osrm.org/nearest/v1/driving"
OSRM_TIMEOUT = 8

BUFFER_DEGREES = 0.0007   # ~78 m clearance around no-fly zone corners

# Number of candidate paths Yen's algorithm generates before selecting
# the time-optimal one.
_YENS_K = 5

LatLon = Tuple[float, float]   # (latitude, longitude)


# ══════════════════════════════════════════════════════════════════════════════
# CORE GEOMETRY
# ══════════════════════════════════════════════════════════════════════════════

def haversine_distance(lat1: float, lon1: float,
                       lat2: float, lon2: float) -> float:
    R = 6_371_000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def distance_3d(lat1: float, lon1: float, alt1: float,
                lat2: float, lon2: float, alt2: float) -> float:
    h = haversine_distance(lat1, lon1, lat2, lon2)
    return math.sqrt(h**2 + (alt2 - alt1)**2)


def calculate_heading(lat1: float, lon1: float,
                      lat2: float, lon2: float) -> float:
    lat1r, lat2r = math.radians(lat1), math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    x = math.sin(dlam) * math.cos(lat2r)
    y = (math.cos(lat1r) * math.sin(lat2r)
         - math.sin(lat1r) * math.cos(lat2r) * math.cos(dlam))
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def interpolate_waypoint(start: Position, end: Position,
                         fraction: float) -> Position:
    return Position(
        latitude  = start.latitude  + (end.latitude  - start.latitude)  * fraction,
        longitude = start.longitude + (end.longitude - start.longitude) * fraction,
        altitude  = start.altitude  + (end.altitude  - start.altitude)  * fraction,
    )


# ══════════════════════════════════════════════════════════════════════════════
# SEGMENT / POLYGON GEOMETRY
# ══════════════════════════════════════════════════════════════════════════════

def _ccw(A: LatLon, B: LatLon, C: LatLon) -> bool:
    return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])


def _segments_intersect(A: LatLon, B: LatLon,
                        C: LatLon, D: LatLon) -> bool:
    return _ccw(A,C,D) != _ccw(B,C,D) and _ccw(A,B,C) != _ccw(A,B,D)


def _segment_crosses_polygon(p1: LatLon, p2: LatLon,
                             polygon: List[LatLon]) -> bool:
    if geofencing.point_in_polygon(p1, polygon): return True
    if geofencing.point_in_polygon(p2, polygon): return True
    n = len(polygon)
    for i in range(n):
        if _segments_intersect(p1, p2, polygon[i], polygon[(i+1) % n]):
            return True
    return False


def _path_hits_no_fly_zone(waypoints: List[LatLon]) -> bool:
    for zone in config.NO_FLY_ZONES:
        poly = zone['polygon']
        for i in range(len(waypoints) - 1):
            if _segment_crosses_polygon(waypoints[i], waypoints[i+1], poly):
                return True
    return False


def _segment_is_clear(p1: LatLon, p2: LatLon) -> bool:
    for zone in config.NO_FLY_ZONES:
        if _segment_crosses_polygon(p1, p2, zone['polygon']):
            return False
    return True


# ══════════════════════════════════════════════════════════════════════════════
# OSRM
# ══════════════════════════════════════════════════════════════════════════════

def fetch_osrm_route(start: Position, goal: Position) -> Optional[List[LatLon]]:
    try:
        url = (f"{OSRM_BASE}/{start.longitude},{start.latitude}"
               f";{goal.longitude},{goal.latitude}"
               f"?overview=full&geometries=geojson")
        print("[OSRM] Requesting direct route...")
        resp = requests.get(url, timeout=OSRM_TIMEOUT)
        data = resp.json()
        if data.get("code") == "Ok" and data.get("routes"):
            coords = data["routes"][0]["geometry"]["coordinates"]
            latlon = [(lat, lon) for lon, lat in coords]
            print(f"[OSRM] ✓ {len(latlon)} road waypoints")
            return latlon
        print(f"[OSRM] No route: {data.get('code','unknown')}")
        return None
    except Exception as e:
        print(f"[OSRM] Direct route failed: {e}")
        return None


def snap_to_road(lat: float, lon: float) -> Optional[LatLon]:
    try:
        url = f"{OSRM_NEAREST}/{lon},{lat}?number=1"
        resp = requests.get(url, timeout=OSRM_TIMEOUT)
        data = resp.json()
        if data.get("code") == "Ok" and data.get("waypoints"):
            loc = data["waypoints"][0]["location"]   # [lon, lat]
            return (loc[1], loc[0])
        return None
    except Exception:
        return None


def fetch_osrm_route_via_positions(positions: List[Position]) -> Optional[List[LatLon]]:
    if len(positions) < 2:
        return None
    try:
        coords = ";".join(f"{p.longitude},{p.latitude}" for p in positions)
        url = f"{OSRM_BASE}/{coords}?overview=full&geometries=geojson"
        print(f"[OSRM] Requesting chained route via {len(positions)} points...")
        resp = requests.get(url, timeout=OSRM_TIMEOUT)
        data = resp.json()
        if data.get("code") == "Ok" and data.get("routes"):
            coords_out = data["routes"][0]["geometry"]["coordinates"]
            latlon = [(lat, lon) for lon, lat in coords_out]
            print(f"[OSRM] ✓ Chained: {len(latlon)} road waypoints")
            return latlon
        print(f"[OSRM] Chained failed: {data.get('code','unknown')}")
        return None
    except Exception as e:
        print(f"[OSRM] Chained error: {e}")
        return None


# ══════════════════════════════════════════════════════════════════════════════
# TURN-SPEED MODEL  (used by build_4d_trajectory waypoint ETA computation)
# ══════════════════════════════════════════════════════════════════════════════

def _turn_speed(prev_heading: float, next_heading: float,
                cruise: float, min_speed: float) -> float:
    """
    Scale cruise speed down proportionally to the sharpness of a turn.

    Turn angle 0°   → full cruise speed  (straight ahead)
    Turn angle 90°  → 50% cruise speed   (right-angle turn)
    Turn angle 180° → min_speed          (U-turn)

    Uses a cosine curve:  speed = cruise * 0.5 * (1 + cos(turn_rad))
    clamped to [min_speed, cruise].
    """
    delta = abs(next_heading - prev_heading) % 360
    if delta > 180:
        delta = 360 - delta
    turn_rad = math.radians(delta)
    scaled   = cruise * 0.5 * (1.0 + math.cos(turn_rad))
    return max(min_speed, scaled)


# ══════════════════════════════════════════════════════════════════════════════
# TIME-COST MODEL  (used by Time-A* graph search)
# ══════════════════════════════════════════════════════════════════════════════

def get_path_travel_time(latlon_path: List[LatLon]) -> float:
    """
    Estimate total flight time for a 2-D road/visibility path.

    For each segment:
      base_time     = segment_dist / turn_speed
      accel_penalty = 2 * (cruise - turn_speed) / DRONE_ACCEL_LIMIT
                      (time to brake into the corner + re-accelerate out)

    The first segment has no incoming heading so no penalty is applied.
    Returns total estimated seconds.
    """
    cruise   = config.DRONE_CRUISE_SPEED
    accel    = config.DRONE_ACCEL_LIMIT
    min_spd  = config.DRONE_MIN_SPEED
    total    = 0.0
    prev_hdg = None

    for i in range(len(latlon_path) - 1):
        p1   = latlon_path[i]
        p2   = latlon_path[i + 1]
        dist = haversine_distance(p1[0], p1[1], p2[0], p2[1])
        if dist < 1e-3:
            continue

        hdg = calculate_heading(p1[0], p1[1], p2[0], p2[1])

        if prev_hdg is None:
            spd     = cruise
            penalty = 0.0
        else:
            spd     = _turn_speed(prev_hdg, hdg, cruise, min_spd)
            delta_v = cruise - spd
            penalty = 2.0 * delta_v / accel if delta_v > 0.1 else 0.0

        total   += dist / spd + penalty
        prev_hdg = hdg

    return total


# ══════════════════════════════════════════════════════════════════════════════
# VISIBILITY GRAPH HELPERS
# ══════════════════════════════════════════════════════════════════════════════

def get_path_travel_time_at_speed(latlon_path: List[LatLon],
                                  cruise_speed: float) -> float:
    """
    Same as get_path_travel_time() but uses an explicit cruise speed
    instead of config.DRONE_CRUISE_SPEED.

    Used by the conflict resolver's Layer-2 Option A to quickly estimate
    whether a speed-reduced trajectory finishes before or after the
    best Yen's alternative (Option B), without rebuilding a full Trajectory4D.

    Turn penalty at the given speed:
        V_corner  = cruise_speed × cos(Δθ / 2)   [clamped to DRONE_MIN_SPEED]
        penalty   = 2 × (cruise_speed − V_corner) / DRONE_ACCEL_LIMIT
    """
    accel   = config.DRONE_ACCEL_LIMIT
    min_spd = config.DRONE_MIN_SPEED
    total   = 0.0
    prev_hdg = None

    for i in range(len(latlon_path) - 1):
        p1   = latlon_path[i]
        p2   = latlon_path[i + 1]
        dist = haversine_distance(p1[0], p1[1], p2[0], p2[1])
        if dist < 1e-3:
            continue

        hdg = calculate_heading(p1[0], p1[1], p2[0], p2[1])

        if prev_hdg is None:
            spd     = cruise_speed
            penalty = 0.0
        else:
            spd     = _turn_speed(prev_hdg, hdg, cruise_speed, min_spd)
            delta_v = cruise_speed - spd
            penalty = 2.0 * delta_v / accel if delta_v > 0.1 else 0.0

        total   += dist / spd + penalty
        prev_hdg = hdg

    return total


def _buffered_vertices(polygon: List[LatLon]) -> List[LatLon]:
    c_lat = sum(v[0] for v in polygon) / len(polygon)
    c_lon = sum(v[1] for v in polygon) / len(polygon)
    result = []
    for lat, lon in polygon:
        dlat, dlon = lat - c_lat, lon - c_lon
        d = math.sqrt(dlat**2 + dlon**2) or 1e-9
        factor = (d + BUFFER_DEGREES) / d
        result.append((c_lat + dlat*factor, c_lon + dlon*factor))
    return result


def _edge_cost_time(p1: LatLon, p2: LatLon) -> float:
    """
    Time-based edge cost: travel time (seconds) at cruise speed.

    Returns float('inf') when the midpoint falls inside a no-fly zone.
    Sensitive-area cost multipliers inflate the travel time proportionally,
    so A* naturally steers around high-cost regions while remaining
    admissible with a DRONE_MAX_SPEED heuristic.
    """
    dist = haversine_distance(p1[0], p1[1], p2[0], p2[1])
    mid  = Position(latitude=(p1[0]+p2[0])/2,
                    longitude=(p1[1]+p2[1])/2, altitude=50.0)
    mult = geofencing.get_position_cost_multiplier(mid)
    if mult >= 999999.0:           # no-fly zone → impassable
        return float('inf')
    return (dist / config.DRONE_CRUISE_SPEED) * mult


def _turn_time_penalty(h1: float, h2: float) -> float:
    """
    Kinematic deceleration/re-acceleration time penalty for a heading change.

    Only applied when |Δθ| > 20° — small course corrections are free.

        V_corner = V_cruise × cos(Δθ / 2)
        τ        = 2 × (V_cruise − V_corner) / DRONE_ACCEL_LIMIT

    The factor of 2 covers the symmetric deceleration into the turn apex
    and the acceleration back to cruise speed.

    This penalty is used in the g-score of the Time-A* graph search to make
    paths with sharp corners genuinely more expensive than smoother routes
    of similar distance.

    Parameters
    ----------
    h1 : incoming heading (degrees, 0–360)
    h2 : outgoing heading (degrees, 0–360)
    """
    delta = abs(h2 - h1) % 360
    if delta > 180:
        delta = 360 - delta                         # normalise to [0°, 180°]
    if delta <= 20.0:                               # below threshold — free
        return 0.0
    v_cruise = config.DRONE_CRUISE_SPEED
    v_corner = v_cruise * math.cos(math.radians(delta / 2.0))
    v_corner = max(config.DRONE_MIN_SPEED, v_corner)
    return 2.0 * (v_cruise - v_corner) / config.DRONE_ACCEL_LIMIT


# ══════════════════════════════════════════════════════════════════════════════
# TIME-OPTIMAL A*  (state-based, with kinematic turn penalties)
# ══════════════════════════════════════════════════════════════════════════════

def _astar_on_graph(
    nodes:          List[LatLon],
    blocked_edges:  set,
    blocked_nodes:  set,
    start_idx:      int,
    goal_idx:       int,
    start_prev_idx: int = -1,
) -> Optional[List[int]]:
    """
    Time-optimal A* over a pre-built visibility graph with kinematic turn
    penalties embedded in the g-score.

    State representation
    --------------------
    State = (current_node_idx, prev_node_idx)

    Tracking the predecessor allows the turn penalty to be computed at
    expansion time without modifying the node set.  prev_node_idx = -1
    at the start node (no incoming heading, no turn penalty for the first
    outgoing edge).

    g-score
    -------
    Cumulative travel time (seconds):
      g += edge_time(cur→nxt)  +  turn_penalty(prev→cur→nxt)

    where turn_penalty is zero when |Δθ| ≤ 20°.

    h-score (admissible lower bound)
    ---------------------------------
      h(n) = haversine(n, goal) / DRONE_MAX_SPEED

    No physical drone can beat this: it assumes straight-line flight at
    maximum possible speed with no turns.

    Parameters
    ----------
    start_prev_idx : the node that precedes start_idx in the root path
                     (used by Yen's to preserve turn-penalty continuity
                      at the spur attachment point).

    Returns
    -------
    Ordered list of node indices from start to goal, or None if unreachable.
    """
    def h(i: int) -> float:
        return (haversine_distance(
            nodes[i][0], nodes[i][1],
            nodes[goal_idx][0], nodes[goal_idx][1],
        ) / config.DRONE_MAX_SPEED)

    INF        = float('inf')
    g_score:   dict = {}          # (cur, prev) → best cumulative time
    came_from: dict = {}          # state → parent_state

    init_state = (start_idx, start_prev_idx)
    g_score[init_state] = 0.0
    heap = [(h(start_idx), 0.0, start_idx, start_prev_idx)]
    closed: set = set()

    while heap:
        f, g, cur, prev = heapq.heappop(heap)
        state = (cur, prev)

        if state in closed:
            continue
        closed.add(state)

        if cur == goal_idx:
            # Reconstruct path — extract the node index from each state
            path: List[int] = []
            s = state
            while s is not None:
                path.append(s[0])
                s = came_from.get(s)
            path.reverse()
            return path

        for nxt in range(len(nodes)):
            if nxt == cur:
                continue
            if nxt in blocked_nodes and nxt != goal_idx:
                continue
            if cur in blocked_nodes and cur != start_idx:
                continue
            if frozenset((cur, nxt)) in blocked_edges:
                continue
            if not _segment_is_clear(nodes[cur], nodes[nxt]):
                continue

            edge_t = _edge_cost_time(nodes[cur], nodes[nxt])
            if edge_t == float('inf'):
                continue

            # Kinematic turn penalty: only when we have an incoming heading
            turn_pen = 0.0
            if prev != -1:
                h_in  = calculate_heading(nodes[prev][0], nodes[prev][1],
                                          nodes[cur][0],  nodes[cur][1])
                h_out = calculate_heading(nodes[cur][0],  nodes[cur][1],
                                          nodes[nxt][0],  nodes[nxt][1])
                turn_pen = _turn_time_penalty(h_in, h_out)

            tg        = g + edge_t + turn_pen
            nxt_state = (nxt, cur)

            if tg < g_score.get(nxt_state, INF):
                g_score[nxt_state]   = tg
                came_from[nxt_state] = state
                heapq.heappush(heap, (tg + h(nxt), tg, nxt, cur))

    return None


# ══════════════════════════════════════════════════════════════════════════════
# YEN'S K-SHORTEST PATHS  →  ALL K TIME-OPTIMAL CANDIDATES
# ══════════════════════════════════════════════════════════════════════════════

def get_k_fastest_paths(
    start: LatLon,
    goal:  LatLon,
    k:     int = _YENS_K,
) -> List[List[LatLon]]:
    """
    Find up to K time-optimal loopless paths using Yen's algorithm wrapped
    around the Time-A* graph search with kinematic turn penalties.

    Algorithm
    ---------
    1. Build visibility-graph node set: start, goal, buffered no-fly zone
       vertices (each expanded outward by BUFFER_DEGREES for safe clearance).
    2. A[0] = primary Time-A* path (lowest g + h cost).
    3. For k = 1 … K−1:
         For each spur node in A[k−1]:
           Block edges used by confirmed paths sharing this root prefix.
           Block all root nodes except the spur node itself.
           Run Time-A* from spur node to goal, using spur's predecessor as
           start_prev_idx so the turn penalty at the re-join point is correct.
           If a spur path is found, add (root + spur) to candidate heap B.
         A[k] = lowest travel-time candidate from B.
    4. Score all confirmed paths with get_path_travel_time() (which also
       includes kinematic corner penalties) and sort ascending.
    5. Return all confirmed paths as List[List[LatLon]], best first.

    The returned paths are used in two ways by plan_trajectory():
      - paths[0]  : best Yen's path (primary if OSRM is unavailable)
      - paths[1:] : alternatives stored as Trajectory4D objects for the
                    conflict resolver's Layer-2 Path-vs-Speed trade-off.

    Returns an empty list if no path exists at all.
    """
    # ── 1. Build visibility-graph node set ───────────────────────────────────
    raw: List[LatLon] = [start, goal]
    for zone in config.NO_FLY_ZONES:
        raw.extend(_buffered_vertices(zone['polygon']))

    seen_keys: set = set()
    nodes: List[LatLon] = []
    for pt in raw:
        key = (round(pt[0], 7), round(pt[1], 7))
        if key not in seen_keys:
            seen_keys.add(key)
            nodes.append(pt)

    # Guarantee start=0, goal=1 after deduplication
    nodes[0] = start
    nodes[1] = goal
    START, GOAL = 0, 1

    # ── 2. A[0]: primary Time-A* path ────────────────────────────────────────
    first_idx = _astar_on_graph(nodes, set(), set(), START, GOAL)
    if first_idx is None:
        print("[Yen's] No base path found")
        return []

    confirmed: List[List[int]] = [first_idx]
    heap_cands: list = []     # (travel_time, uid, path_indices)
    _uid = 0

    t0 = get_path_travel_time([nodes[i] for i in first_idx])
    print(f"[Yen's] k=0: {len(first_idx)} nodes, time={t0:.1f}s")

    # ── 3. Yen's main loop ────────────────────────────────────────────────────
    for k_iter in range(1, k):
        prev_path = confirmed[k_iter - 1]

        for spur_pos in range(len(prev_path) - 1):
            spur_node = prev_path[spur_pos]
            root_nodes = prev_path[:spur_pos + 1]
            # Predecessor of spur node in the root — needed for turn penalty
            spur_prev = prev_path[spur_pos - 1] if spur_pos > 0 else -1

            blocked_edges: set = set()
            blocked_nodes: set = set()

            # Block edges departing spur_node that are used by any confirmed
            # path whose root prefix matches this root.
            for cp in confirmed:
                if (len(cp) > spur_pos
                        and cp[:spur_pos + 1] == root_nodes):
                    blocked_edges.add(
                        frozenset((cp[spur_pos], cp[spur_pos + 1]))
                    )
            # Same for current heap candidates
            for _, _, cand_path in heap_cands:
                if (len(cand_path) > spur_pos
                        and cand_path[:spur_pos + 1] == root_nodes):
                    blocked_edges.add(
                        frozenset((cand_path[spur_pos], cand_path[spur_pos + 1]))
                    )

            # Block root nodes (except the spur itself) to enforce looplessness
            for ni in root_nodes[:-1]:
                blocked_nodes.add(ni)

            spur_path = _astar_on_graph(
                nodes, blocked_edges, blocked_nodes, spur_node, GOAL,
                start_prev_idx=spur_prev,   # preserves turn-penalty continuity
            )

            if spur_path is not None:
                full_path = root_nodes[:-1] + spur_path

                already_confirmed  = any(full_path == cp for cp in confirmed)
                already_candidate  = any(full_path == cp for _, _, cp in heap_cands)

                if not already_confirmed and not already_candidate:
                    cost = get_path_travel_time([nodes[i] for i in full_path])
                    heapq.heappush(heap_cands, (cost, _uid, full_path))
                    _uid += 1

        if not heap_cands:
            print(f"[Yen's] No more candidates after k={k_iter}")
            break

        _, _, best_cand = heapq.heappop(heap_cands)
        confirmed.append(best_cand)
        t_c = get_path_travel_time([nodes[i] for i in best_cand])
        print(f"[Yen's] k={k_iter}: {len(best_cand)} nodes, time={t_c:.1f}s")

    # ── 4. Score and sort all confirmed paths by travel time ──────────────────
    result_paths: List[List[LatLon]] = []
    scored = []
    for idx_path in confirmed:
        latlon = [nodes[i] for i in idx_path]
        t = get_path_travel_time(latlon)
        dist = sum(
            haversine_distance(latlon[j][0], latlon[j][1],
                               latlon[j+1][0], latlon[j+1][1])
            for j in range(len(latlon) - 1)
        )
        scored.append((t, latlon))
        print(f"[Yen's] Candidate {len(scored)}: "
              f"{len(latlon)} nodes, dist={dist:.0f}m, time={t:.1f}s")

    scored.sort(key=lambda x: x[0])

    if scored:
        print(f"[Yen's] ✓ Time-optimal: {len(scored[0][1])} nodes, "
              f"time={scored[0][0]:.1f}s")

    result_paths = [latlon for _, latlon in scored]
    return result_paths


def visibility_graph_astar(start: LatLon, goal: LatLon) -> Optional[List[LatLon]]:
    """
    Compatibility wrapper — returns the single fastest path.
    New code should call get_k_fastest_paths() directly.
    """
    paths = get_k_fastest_paths(start, goal, k=_YENS_K)
    return paths[0] if paths else None


# ══════════════════════════════════════════════════════════════════════════════
# PATH POST-PROCESSING
# ══════════════════════════════════════════════════════════════════════════════

def _thin_path(points: List[LatLon], min_dist_m: float = 25.0) -> List[LatLon]:
    if len(points) < 2:
        return points
    result = [points[0]]
    for pt in points[1:-1]:
        if haversine_distance(result[-1][0], result[-1][1], pt[0], pt[1]) >= min_dist_m:
            result.append(pt)
    result.append(points[-1])
    return result


# ══════════════════════════════════════════════════════════════════════════════
# DIRECTION + STRATUM ASSIGNMENT
# ══════════════════════════════════════════════════════════════════════════════

def bearing_to_direction(heading: float) -> str:
    h = heading % 360
    if 337.5 <= h or h < 22.5:   return 'NORTH'
    if 22.5  <= h < 67.5:         return 'DIAGONAL'
    if 67.5  <= h < 112.5:        return 'EAST'
    if 112.5 <= h < 157.5:        return 'DIAGONAL'
    if 157.5 <= h < 202.5:        return 'SOUTH'
    if 202.5 <= h < 247.5:        return 'DIAGONAL'
    if 247.5 <= h < 292.5:        return 'WEST'
    return 'DIAGONAL'


def direction_to_stratum(direction: str) -> int:
    return config.DIRECTION_STRATA.get(direction, 80)


# ══════════════════════════════════════════════════════════════════════════════
# 4-D TRAJECTORY BUILDER
# ══════════════════════════════════════════════════════════════════════════════

def _is_in_terminal_area(lat: float, lon: float, alt: float,
                          pad_lat: float, pad_lon: float) -> bool:
    h = haversine_distance(lat, lon, pad_lat, pad_lon)
    return h <= config.TERMINAL_RADIUS and alt <= config.TERMINAL_HEIGHT


def _compute_sigma_t(cumulative_arc_m: float) -> float:
    return config.CPA_SIGMA_0 + config.CPA_DRIFT_K * cumulative_arc_m


def build_4d_trajectory(latlon_path: List[LatLon],
                        pickup_pos:   Position,
                        delivery_pos: Position,
                        stratum:      int,
                        direction:    str,
                        start_time:   float,
                        cruise_speed_override: Optional[float] = None) -> Trajectory4D:
    """
    Convert a 2-D road path into a full 4-D trajectory.

    Synthetic waypoints are inserted for GROUND / TERMINAL / CLIMB /
    CRUISE / DESCENT phases.  CRUISE waypoints use turn-adjusted speeds
    (cosine model) so that ETA and sigma_t values are accurate for CPA.

    Parameters
    ----------
    cruise_speed_override : if provided, overrides config.DRONE_CRUISE_SPEED
        for all CRUISE phase segments and their kinematic turn penalties.
        Used by the conflict resolver's Layer-2 Option A (speed reduction)
        to build a candidate trajectory at a reduced speed and compare its
        final ETA against Option B's alternative paths.
    """
    cruise_speed  = cruise_speed_override if cruise_speed_override is not None \
                    else config.DRONE_CRUISE_SPEED
    climb_rate    = config.DRONE_CLIMB_RATE
    descent_rate  = config.DRONE_DESCENT_RATE

    climb_time     = stratum / climb_rate
    descent_time   = stratum / descent_rate
    climb_h_dist   = cruise_speed * climb_time
    descent_h_dist = cruise_speed * descent_time

    def walk_path_forward(path: List[LatLon], target_dist: float
                          ) -> Tuple[LatLon, int, float]:
        d = 0.0
        for i in range(len(path) - 1):
            seg = haversine_distance(path[i][0], path[i][1],
                                     path[i+1][0], path[i+1][1])
            if d + seg >= target_dist:
                t = (target_dist - d) / seg
                lat = path[i][0] + t * (path[i+1][0] - path[i][0])
                lon = path[i][1] + t * (path[i+1][1] - path[i][1])
                return (lat, lon), i, 0.0
            d += seg
        return path[-1], len(path)-1, target_dist - d

    def walk_path_backward(path: List[LatLon], target_dist: float
                           ) -> Tuple[LatLon, int]:
        d = 0.0
        for i in range(len(path)-1, 0, -1):
            seg = haversine_distance(path[i][0], path[i][1],
                                     path[i-1][0], path[i-1][1])
            if d + seg >= target_dist:
                t = (target_dist - d) / seg
                lat = path[i][0] + t * (path[i-1][0] - path[i][0])
                lon = path[i][1] + t * (path[i-1][1] - path[i][1])
                return (lat, lon), i
            d += seg
        return path[0], 0

    total_path_len = sum(
        haversine_distance(latlon_path[i][0], latlon_path[i][1],
                           latlon_path[i+1][0], latlon_path[i+1][1])
        for i in range(len(latlon_path)-1)
    )

    max_one_side   = total_path_len * 0.4
    climb_h_dist   = min(climb_h_dist,   max_one_side)
    descent_h_dist = min(descent_h_dist, max_one_side)

    climb_end_pt,     climb_end_idx,    _ = walk_path_forward(latlon_path, climb_h_dist)
    descent_start_pt, descent_start_idx   = walk_path_backward(latlon_path, descent_h_dist)

    if climb_end_idx >= descent_start_idx:
        mid_lat = (latlon_path[0][0] + latlon_path[-1][0]) / 2
        mid_lon = (latlon_path[0][1] + latlon_path[-1][1]) / 2
        climb_end_pt      = (mid_lat, mid_lon)
        descent_start_pt  = (mid_lat, mid_lon)
        climb_end_idx     = len(latlon_path) // 2
        descent_start_idx = climb_end_idx

    segments: List[Tuple[float, float, float, FlightPhase]] = []

    pick_lat  = pickup_pos.latitude
    pick_lon  = pickup_pos.longitude
    deliv_lat = delivery_pos.latitude
    deliv_lon = delivery_pos.longitude

    segments.append((pick_lat, pick_lon, 0.0, FlightPhase.GROUND))

    N_CLIMB = 4
    for k in range(1, N_CLIMB + 1):
        t   = k / N_CLIMB
        lat = pick_lat + t * (climb_end_pt[0] - pick_lat)
        lon = pick_lon + t * (climb_end_pt[1] - pick_lon)
        alt = t * stratum
        phase = (FlightPhase.TERMINAL
                 if _is_in_terminal_area(lat, lon, alt, pick_lat, pick_lon)
                 else FlightPhase.CLIMB)
        segments.append((lat, lon, alt, phase))

    for lat, lon in latlon_path[climb_end_idx+1 : descent_start_idx]:
        segments.append((lat, lon, float(stratum), FlightPhase.CRUISE))

    N_DESCENT = 4
    for k in range(1, N_DESCENT + 1):
        t   = k / N_DESCENT
        lat = descent_start_pt[0] + t * (deliv_lat - descent_start_pt[0])
        lon = descent_start_pt[1] + t * (deliv_lon - descent_start_pt[1])
        alt = stratum * (1.0 - t)
        phase = (FlightPhase.TERMINAL
                 if _is_in_terminal_area(lat, lon, alt, deliv_lat, deliv_lon)
                 else FlightPhase.DESCENT)
        segments.append((lat, lon, alt, phase))

    segments.append((deliv_lat, deliv_lon, 0.0, FlightPhase.GROUND))

    waypoints4d:   List[Waypoint4D] = []
    current_time   = start_time
    cumulative_arc = 0.0
    total_dist     = 0.0

    prev_lat, prev_lon, prev_alt = segments[0][:3]

    for i, (lat, lon, alt, phase) in enumerate(segments):

        if i == 0:
            overall_heading = calculate_heading(
                segments[0][0], segments[0][1],
                segments[-1][0], segments[-1][1]
            )
            sigma    = _compute_sigma_t(0.0)
            wp_xyz   = anchor_registry._to_xyz(lat, lon, alt)
            bindings, _, _ = anchor_registry.compute_anchor_bindings(wp_xyz)
            waypoints4d.append(Waypoint4D(
                position=Position(latitude=lat, longitude=lon, altitude=alt),
                eta=current_time,
                speed=cruise_speed,
                heading=overall_heading,
                phase=phase,
                sigma_t=sigma,
                anchor_bindings=bindings,
            ))
            continue

        seg_3d = distance_3d(prev_lat, prev_lon, prev_alt, lat, lon, alt)
        seg_2d = haversine_distance(prev_lat, prev_lon, lat, lon)
        cumulative_arc += seg_2d
        total_dist     += seg_3d

        heading = calculate_heading(prev_lat, prev_lon, lat, lon)

        if phase == FlightPhase.CLIMB and alt > prev_alt:
            speed = climb_rate
        elif phase == FlightPhase.DESCENT and alt < prev_alt:
            speed = descent_rate
        elif phase == FlightPhase.GROUND:
            speed = 0.0
        elif phase == FlightPhase.CRUISE:
            prev_heading = waypoints4d[-1].heading if waypoints4d else heading
            speed = _turn_speed(
                prev_heading, heading,
                cruise_speed, config.DRONE_MIN_SPEED
            )
        else:
            speed = cruise_speed   # TERMINAL

        if phase in (FlightPhase.CLIMB, FlightPhase.TERMINAL) and alt > prev_alt:
            seg_time = (alt - prev_alt) / climb_rate
        elif phase in (FlightPhase.DESCENT, FlightPhase.TERMINAL) and alt < prev_alt:
            seg_time = (prev_alt - alt) / descent_rate
        elif phase == FlightPhase.GROUND:
            seg_time = 0.5
        else:
            seg_time = seg_3d / speed   # turn-adjusted for CRUISE

        current_time += seg_time
        sigma         = _compute_sigma_t(cumulative_arc)

        wp_xyz   = anchor_registry._to_xyz(lat, lon, min(alt, 130.0))
        bindings, _, _ = anchor_registry.compute_anchor_bindings(wp_xyz)

        waypoints4d.append(Waypoint4D(
            position=Position(latitude=lat, longitude=lon, altitude=min(alt, 130.0)),
            eta=current_time,
            speed=speed,
            heading=heading,
            phase=phase,
            sigma_t=sigma,
            anchor_bindings=bindings,
        ))

        prev_lat, prev_lon, prev_alt = lat, lon, alt

    waypoints4d[-1].speed = 0.0
    total_time    = waypoints4d[-1].eta - waypoints4d[0].eta
    battery_usage = (total_time * config.DRONE_POWER_CONSUMPTION
                     / config.DRONE_BATTERY_CAPACITY * 100)

    return Trajectory4D(
        waypoints=waypoints4d,
        total_distance=total_dist,
        total_time=total_time,
        estimated_battery_usage=battery_usage,
        takeoff_delay=0.0,
        base_start_time=start_time,
        stratum=stratum,
        direction=direction,
    )


# ══════════════════════════════════════════════════════════════════════════════
# PUBLIC: BLOCKED-PATH VISUALISATION
# ══════════════════════════════════════════════════════════════════════════════

def get_direct_path(start: Position, goal: Position) -> List[LatLon]:
    """Straight line between start and goal — used only for reroute animation."""
    return [(start.latitude, start.longitude),
            (goal.latitude,  goal.longitude)]


# ══════════════════════════════════════════════════════════════════════════════
# PUBLIC: MAIN TRAJECTORY PLANNER
# ══════════════════════════════════════════════════════════════════════════════

def plan_trajectory(
    start:      Position,
    goal:       Position,
    start_time: float,
    stratum:    Optional[int] = None,
    direction:  Optional[str] = None,
) -> Optional[Tuple[Trajectory4D, List[Trajectory4D]]]:
    """
    Plan a road-following, obstacle-aware, time-optimal 4-D trajectory.

    Returns
    -------
    (primary_trajectory, alternative_trajectories)
      primary_trajectory      : best trajectory (OSRM or Yen's)
      alternative_trajectories: up to K−1 Yen's alternative Trajectory4D
                                objects, sorted by travel time ascending.
                                Used by the conflict resolver's Layer-2
                                Path-vs-Speed trade-off.
    Returns None if no valid path can be found at all.

    Routing Levels
    --------------
    Level 1: Direct OSRM route (validated against no-fly zones)
    Level 2: Yen's bypass corners snapped to roads, chained OSRM route
    Level 3: Pure Yen's K=5 visibility graph (last resort)

    Yen's K paths are always computed — the conflict resolver needs paths
    2-5 as pre-built alternatives regardless of which primary-path level
    succeeds.
    """
    print(f"[Pathfinding] ({start.latitude:.4f},{start.longitude:.4f}) -> "
          f"({goal.latitude:.4f},{goal.longitude:.4f})")

    # ── Always compute K candidate paths (needed for alternatives) ────────────
    k_paths = get_k_fastest_paths(
        (start.latitude, start.longitude),
        (goal.latitude,  goal.longitude),
        k=_YENS_K,
    )

    latlon_path: Optional[List[LatLon]] = None

    # ── Level 1: Direct OSRM ─────────────────────────────────────────────────
    road_points = fetch_osrm_route(start, goal)

    if road_points and not _path_hits_no_fly_zone(road_points):
        print("[Pathfinding] ✓ Level 1: OSRM direct route clear")
        latlon_path = road_points

    elif road_points:
        print("[Pathfinding] OSRM route blocked — trying road bypass...")
        raw_bypass = k_paths[0] if k_paths else None

        if raw_bypass and len(raw_bypass) > 2:
            interior = raw_bypass[1:-1]
            snapped: List[Position] = []
            all_snapped = True
            for lat, lon in interior:
                road_pt = snap_to_road(lat, lon)
                if road_pt:
                    snapped.append(Position(latitude=road_pt[0],
                                            longitude=road_pt[1],
                                            altitude=start.altitude))
                else:
                    all_snapped = False
                    break

            if all_snapped and snapped:
                via = [start] + snapped + [goal]
                chained = fetch_osrm_route_via_positions(via)
                if chained and not _path_hits_no_fly_zone(chained):
                    print(f"[Pathfinding] ✓ Level 2: road bypass clear "
                          f"({len(snapped)} bypass points)")
                    latlon_path = chained

        if latlon_path is None:
            print("[Pathfinding] Level 3: Yen's K=5 visibility graph")
            latlon_path = k_paths[0] if k_paths else None

    else:
        print("[Pathfinding] OSRM unavailable — Level 3 Yen's K=5")
        latlon_path = k_paths[0] if k_paths else None

    if latlon_path is None:
        print("[Pathfinding] No valid path found")
        return None

    latlon_path = _thin_path(latlon_path, min_dist_m=25)

    # ── Assign stratum from heading if not provided ───────────────────────────
    if direction is None:
        overall_heading = calculate_heading(
            latlon_path[0][0], latlon_path[0][1],
            latlon_path[-1][0], latlon_path[-1][1]
        )
        direction = bearing_to_direction(overall_heading)

    if stratum is None:
        stratum = direction_to_stratum(direction)

    # ── Build primary trajectory ──────────────────────────────────────────────
    primary = build_4d_trajectory(latlon_path, start, goal,
                                   stratum, direction, start_time)

    # ── Build alternative trajectories from ALL Yen's paths ──────────────────
    # All K Yen's paths are included — the resolver picks the best clear one.
    # Paths are already sorted by travel time (fastest first) from
    # get_k_fastest_paths(), so the resolver sees them in priority order.
    alternatives: List[Trajectory4D] = []
    for alt_latlon in k_paths:
        thinned = _thin_path(alt_latlon, min_dist_m=25)
        try:
            alt_traj = build_4d_trajectory(
                thinned, start, goal, stratum, direction, start_time
            )
            alternatives.append(alt_traj)
        except Exception as e:
            print(f"[Pathfinding] Alt trajectory build failed: {e}")

    print(f"[Pathfinding] Generated {len(alternatives)} alternative trajectories "
          f"for conflict resolution")

    return primary, alternatives
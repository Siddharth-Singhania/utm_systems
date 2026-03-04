"""
Pathfinding + 4-D Trajectory Builder

Public API
----------
get_direct_path(start, goal)          -> List[LatLon]     (straight line, for reroute viz)
plan_trajectory(start, goal,
                start_time,
                stratum, direction)   -> Optional[Trajectory4D]

Internal pipeline
-----------------
Level 1  OSRM direct road route        validated against no-fly zones
Level 2  OSRM blocked -> Yen's K=5 visibility graph bypass,
         corners snapped to nearest road, chained OSRM route
Level 3  Pure Yen's K=5 visibility graph A*  (last resort, no road guarantee)

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

Time-Optimal Routing (Yen's K=5)
---------------------------------
visibility_graph_astar() generates up to K=5 loopless candidate paths,
scores each with get_path_travel_time() (segment time + turn accel penalty),
and returns the time-optimal candidate rather than the shortest-distance one.
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
# TURN-SPEED MODEL
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
# TIME-COST MODEL
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


def _edge_cost(p1: LatLon, p2: LatLon) -> float:
    dist = haversine_distance(p1[0], p1[1], p2[0], p2[1])
    mid  = Position(latitude=(p1[0]+p2[0])/2,
                    longitude=(p1[1]+p2[1])/2, altitude=50.0)
    return dist * geofencing.get_position_cost_multiplier(mid)


# ══════════════════════════════════════════════════════════════════════════════
# LOW-LEVEL A* ON A PRE-BUILT NODE SET  (inner loop for Yen's)
# ══════════════════════════════════════════════════════════════════════════════

def _astar_on_graph(
    nodes:         List[LatLon],
    blocked_edges: set,
    blocked_nodes: set,
    start_idx:     int,
    goal_idx:      int,
) -> Optional[List[int]]:
    """
    A* over a visibility graph defined by `nodes`.

    An edge (i→j) is traversable if:
      - neither i nor j is in blocked_nodes
      - frozenset({i,j}) is not in blocked_edges
      - the straight line between nodes[i] and nodes[j] clears all no-fly zones

    Returns an ordered list of node indices, or None if unreachable.
    """
    n = len(nodes)

    def h(i: int) -> float:
        return haversine_distance(
            nodes[i][0], nodes[i][1],
            nodes[goal_idx][0], nodes[goal_idx][1]
        )

    INF       = float('inf')
    g_score   = [INF] * n
    came_from = [-1]  * n
    g_score[start_idx] = 0.0
    heap   = [(h(start_idx), 0.0, start_idx)]
    closed: set = set()

    while heap:
        f, g, cur = heapq.heappop(heap)
        if cur in closed:
            continue
        closed.add(cur)

        if cur == goal_idx:
            path: List[int] = []
            idx = goal_idx
            while idx != -1:
                path.append(idx)
                idx = came_from[idx]
            path.reverse()
            return path

        for nxt in range(n):
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

            tg = g + _edge_cost(nodes[cur], nodes[nxt])
            if tg < g_score[nxt]:
                g_score[nxt]   = tg
                came_from[nxt] = cur
                heapq.heappush(heap, (tg + h(nxt), tg, nxt))

    return None


# ══════════════════════════════════════════════════════════════════════════════
# YEN'S K-SHORTEST PATHS  →  TIME-OPTIMAL SELECTION
# ══════════════════════════════════════════════════════════════════════════════

def visibility_graph_astar(start: LatLon, goal: LatLon) -> Optional[List[LatLon]]:
    """
    Find the TIME-OPTIMAL path between start and goal using Yen's K-shortest
    loopless paths algorithm (K = _YENS_K) on the visibility graph.

    Steps
    -----
    1. Build node set: start, goal, buffered no-fly zone vertices.
    2. A[0] = shortest path by geofencing-weighted edge cost.
    3. For k = 1 … K-1:
         For each spur node in A[k-1]:
           Block edges used by confirmed paths with the same root prefix.
           Block all root nodes except the spur node.
           Run _astar_on_graph from spur node to goal.
           Add root + spur to candidate heap B if not already seen.
         A[k] = lowest-cost candidate from B.
    4. Score all confirmed paths with get_path_travel_time().
    5. Return the path with the lowest travel time.
    """
    # ── 1. Build node set ─────────────────────────────────────────────────────
    raw_nodes: List[LatLon] = [start, goal]
    for zone in config.NO_FLY_ZONES:
        raw_nodes.extend(_buffered_vertices(zone['polygon']))

    seen_keys: set = set()
    nodes: List[LatLon] = []
    for pt in raw_nodes:
        key = (round(pt[0], 7), round(pt[1], 7))
        if key not in seen_keys:
            seen_keys.add(key)
            nodes.append(pt)

    nodes[0] = start
    nodes[1] = goal
    START, GOAL = 0, 1

    # ── 2. Find A[0] ──────────────────────────────────────────────────────────
    first = _astar_on_graph(nodes, set(), set(), START, GOAL)
    if first is None:
        print("[Yen's] No base path found")
        return None

    confirmed: List[List[int]] = [first]
    candidates: list = []   # heap of (edge_cost, uid, path_indices)
    _uid = 0

    print(f"[Yen's] Base path: {len(first)} nodes, "
          f"time={get_path_travel_time([nodes[i] for i in first]):.1f}s")

    # ── 3. Yen's main loop ────────────────────────────────────────────────────
    for k in range(1, _YENS_K):
        prev_path = confirmed[k - 1]

        for spur_idx in range(len(prev_path) - 1):
            spur_node  = prev_path[spur_idx]
            root_nodes = prev_path[:spur_idx + 1]

            blocked_edges: set = set()
            blocked_nodes: set = set()

            # Block edges used by confirmed paths sharing this root prefix
            for cp in confirmed:
                if (len(cp) > spur_idx and
                        cp[:spur_idx + 1] == root_nodes):
                    blocked_edges.add(
                        frozenset((cp[spur_idx], cp[spur_idx + 1]))
                    )

            # Block edges used by candidates sharing this root prefix
            for _, _, cand_path in candidates:
                if (len(cand_path) > spur_idx and
                        cand_path[:spur_idx + 1] == root_nodes):
                    blocked_edges.add(
                        frozenset((cand_path[spur_idx], cand_path[spur_idx + 1]))
                    )

            # Block all root nodes except the spur node itself
            for node_idx in root_nodes[:-1]:
                blocked_nodes.add(node_idx)

            spur_path = _astar_on_graph(
                nodes, blocked_edges, blocked_nodes, spur_node, GOAL
            )

            if spur_path is not None:
                full_path = root_nodes[:-1] + spur_path

                already_confirmed = any(full_path == cp for cp in confirmed)
                already_candidate = any(full_path == cp for _, _, cp in candidates)

                if not already_confirmed and not already_candidate:
                    cost = sum(
                        _edge_cost(nodes[full_path[j]], nodes[full_path[j + 1]])
                        for j in range(len(full_path) - 1)
                    )
                    heapq.heappush(candidates, (cost, _uid, full_path))
                    _uid += 1

        if not candidates:
            print(f"[Yen's] No more candidates after k={k}, stopping early")
            break

        _, _, best_cand = heapq.heappop(candidates)
        confirmed.append(best_cand)

        travel_t = get_path_travel_time([nodes[i] for i in best_cand])
        print(f"[Yen's] k={k}: {len(best_cand)} nodes, time={travel_t:.1f}s")

    # ── 4. Score all confirmed paths by travel time ────────────────────────────
    scored = []
    for idx_path in confirmed:
        latlon = [nodes[i] for i in idx_path]
        t      = get_path_travel_time(latlon)
        dist   = sum(
            haversine_distance(
                latlon[j][0], latlon[j][1],
                latlon[j+1][0], latlon[j+1][1]
            )
            for j in range(len(latlon) - 1)
        )
        scored.append((t, latlon))
        print(f"[Yen's] Candidate {len(scored)}: "
              f"{len(latlon)} nodes, dist={dist:.0f}m, time={t:.1f}s")

    scored.sort(key=lambda x: x[0])
    best_time, best_path = scored[0]
    print(f"[Yen's] ✓ Time-optimal: {len(best_path)} nodes, "
          f"time={best_time:.1f}s")

    return best_path


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
                        start_time:   float) -> Trajectory4D:
    """
    Convert a 2-D road path into a full 4-D trajectory.

    Synthetic waypoints are inserted for GROUND / TERMINAL / CLIMB /
    CRUISE / DESCENT phases.  CRUISE waypoints use turn-adjusted speeds
    (cosine model) so that ETA and sigma_t values are accurate for CPA.
    """
    cruise_speed  = config.DRONE_CRUISE_SPEED
    climb_rate    = config.DRONE_CLIMB_RATE
    descent_rate  = config.DRONE_DESCENT_RATE

    climb_time     = stratum / climb_rate
    descent_time   = stratum / descent_rate
    climb_h_dist   = cruise_speed * climb_time
    descent_h_dist = cruise_speed * descent_time

    # ── Walk path to find climb endpoint and descent startpoint ──────────────

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

    # ── Build (lat, lon, alt, phase) segments ────────────────────────────────
    segments: List[Tuple[float, float, float, FlightPhase]] = []

    pick_lat  = pickup_pos.latitude
    pick_lon  = pickup_pos.longitude
    deliv_lat = delivery_pos.latitude
    deliv_lon = delivery_pos.longitude

    # 1. Launch pad
    segments.append((pick_lat, pick_lon, 0.0, FlightPhase.GROUND))

    # 2. Climb
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

    # 3. Cruise
    for lat, lon in latlon_path[climb_end_idx+1 : descent_start_idx]:
        segments.append((lat, lon, float(stratum), FlightPhase.CRUISE))

    # 4. Descent
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

    # 5. Delivery pad
    segments.append((deliv_lat, deliv_lon, 0.0, FlightPhase.GROUND))

    # ── Compute ETAs, sigma_t, anchor bindings ────────────────────────────────
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

        # ── Heading ───────────────────────────────────────────────────────────
        heading = calculate_heading(prev_lat, prev_lon, lat, lon)

        # ── Speed (phase-aware; CRUISE slows on turns) ────────────────────────
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

        # ── Segment travel time ───────────────────────────────────────────────
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

def plan_trajectory(start: Position, goal: Position,
                    start_time: float,
                    stratum: Optional[int] = None,
                    direction: Optional[str] = None) -> Optional[Trajectory4D]:
    """
    Plan a road-following, obstacle-aware, time-optimal 4-D trajectory.

    Level 1: Direct OSRM route (validated against no-fly zones)
    Level 2: Yen's K=5 bypass corners snapped to roads, chained OSRM route
    Level 3: Pure Yen's K=5 visibility graph (no road guarantee)

    Stratum and direction are derived from overall heading if not provided.
    """
    print(f"[Pathfinding] ({start.latitude:.4f},{start.longitude:.4f}) -> "
          f"({goal.latitude:.4f},{goal.longitude:.4f})")

    latlon_path: Optional[List[LatLon]] = None

    # ── Level 1: Direct OSRM ──────────────────────────────────────────────────
    road_points = fetch_osrm_route(start, goal)

    if road_points and not _path_hits_no_fly_zone(road_points):
        print("[Pathfinding] ✓ Level 1: OSRM direct route clear")
        latlon_path = road_points

    elif road_points:
        print("[Pathfinding] OSRM route blocked — trying road bypass...")
        raw_bypass = visibility_graph_astar(
            (start.latitude, start.longitude),
            (goal.latitude,  goal.longitude)
        )

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
            latlon_path = visibility_graph_astar(
                (start.latitude, start.longitude),
                (goal.latitude,  goal.longitude)
            )

    else:
        print("[Pathfinding] OSRM unavailable — Level 3 Yen's K=5")
        latlon_path = visibility_graph_astar(
            (start.latitude, start.longitude),
            (goal.latitude,  goal.longitude)
        )

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

    return build_4d_trajectory(latlon_path, start, goal,
                               stratum, direction, start_time)
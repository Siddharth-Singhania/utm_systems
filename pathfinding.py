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
Level 2  OSRM blocked -> A* bypass corners snapped to nearest road
Level 3  Pure visibility-graph A*      (last resort, no road guarantee)

After a 2-D road path is found, build_4d_trajectory() converts it into a
full Trajectory4D with:
  - GROUND waypoint at z=0 on the launch pad
  - TERMINAL/CLIMB synthetic waypoints (ascending to stratum)
  - CRUISE waypoints (OSRM road points at stratum altitude)
  - DESCENT/TERMINAL synthetic waypoints (descending to delivery pad)
  - GROUND waypoint at z=0 on the delivery pad
  - sigma_t accumulated per waypoint (ETA uncertainty cone)
  - phase tag on every waypoint
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

LatLon = Tuple[float, float]   # (latitude, longitude)


# ══════════════════════════════════════════════════════════════════════════════
# CORE GEOMETRY  (also imported by conflict_detection.py)
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
# VISIBILITY GRAPH A*
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


def visibility_graph_astar(start: LatLon, goal: LatLon) -> Optional[List[LatLon]]:
    node_set: List[LatLon] = [start, goal]
    for zone in config.NO_FLY_ZONES:
        node_set.extend(_buffered_vertices(zone['polygon']))

    seen: set = set()
    nodes: List[LatLon] = []
    for pt in node_set:
        key = (round(pt[0], 7), round(pt[1], 7))
        if key not in seen:
            seen.add(key)
            nodes.append(pt)

    nodes[0] = start
    nodes[1] = goal
    n = len(nodes)

    def h(i: int) -> float:
        return haversine_distance(nodes[i][0], nodes[i][1], goal[0], goal[1])

    INF = float('inf')
    g_score  = [INF] * n;  g_score[0] = 0.0
    came_from = [-1] * n
    heap = [(h(0), 0.0, 0)]
    closed: set = set()

    while heap:
        f, g, cur = heapq.heappop(heap)
        if cur in closed:
            continue
        closed.add(cur)
        if cur == 1:
            path: List[LatLon] = []
            idx = 1
            while idx != -1:
                path.append(nodes[idx])
                idx = came_from[idx]
            path.reverse()
            print(f"[A*] ✓ {len(path)}-node path found")
            return path
        for nxt in range(n):
            if nxt in closed or not _segment_is_clear(nodes[cur], nodes[nxt]):
                continue
            tg = g + _edge_cost(nodes[cur], nodes[nxt])
            if tg < g_score[nxt]:
                g_score[nxt] = tg
                came_from[nxt] = cur
                heapq.heappush(heap, (tg + h(nxt), tg, nxt))

    print("[A*] No path found")
    return None


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
    """Map heading (0-360) to cardinal direction string."""
    h = heading % 360
    if 337.5 <= h or h < 22.5:
        return 'NORTH'
    if 22.5 <= h < 67.5:
        return 'DIAGONAL'
    if 67.5 <= h < 112.5:
        return 'EAST'
    if 112.5 <= h < 157.5:
        return 'DIAGONAL'
    if 157.5 <= h < 202.5:
        return 'SOUTH'
    if 202.5 <= h < 247.5:
        return 'DIAGONAL'
    if 247.5 <= h < 292.5:
        return 'WEST'
    return 'DIAGONAL'


def direction_to_stratum(direction: str) -> int:
    """Return the primary stratum (metres) for a given direction."""
    return config.DIRECTION_STRATA.get(direction, 80)


# ══════════════════════════════════════════════════════════════════════════════
# 4-D TRAJECTORY BUILDER
# ══════════════════════════════════════════════════════════════════════════════

def _is_in_terminal_area(lat: float, lon: float, alt: float,
                          pad_lat: float, pad_lon: float) -> bool:
    """True if point is within the terminal cylinder of a pad."""
    h = haversine_distance(lat, lon, pad_lat, pad_lon)
    return h <= config.TERMINAL_RADIUS and alt <= config.TERMINAL_HEIGHT


def _compute_sigma_t(cumulative_arc_m: float) -> float:
    """Timing uncertainty at a given arc-distance from launch."""
    return config.CPA_SIGMA_0 + config.CPA_DRIFT_K * cumulative_arc_m


def build_4d_trajectory(latlon_path: List[LatLon],
                        pickup_pos:   Position,
                        delivery_pos: Position,
                        stratum:      int,
                        direction:    str,
                        start_time:   float) -> Trajectory4D:
    """
    Convert a 2-D road path into a full 4-D trajectory.

    Synthetic waypoints are inserted for:
      - Launch pad (z=0, GROUND)
      - Climb from z=0 to stratum (TERMINAL then CLIMB)
      - Cruise at stratum (CRUISE)
      - Descent from stratum to z=0 (DESCENT then TERMINAL)
      - Delivery pad (z=0, GROUND)

    sigma_t is accumulated per waypoint based on cumulative arc-length.
    The drone travels horizontally while climbing/descending — the horizontal
    distance covered during climb/descent is sliced off the first/last
    road segments so waypoints remain on the route.
    """
    cruise_speed  = config.DRONE_CRUISE_SPEED
    climb_rate    = config.DRONE_CLIMB_RATE
    descent_rate  = config.DRONE_DESCENT_RATE

    # Time to reach cruise stratum from ground (and back)
    climb_time   = stratum / climb_rate      # seconds
    descent_time = stratum / descent_rate    # seconds

    # Horizontal distance covered during climb/descent at cruise speed
    climb_h_dist   = cruise_speed * climb_time    # metres
    descent_h_dist = cruise_speed * descent_time  # metres

    # ── Walk road path to find climb endpoint and descent startpoint ──────────
    # We advance along the path until we've covered climb_h_dist metres —
    # that point is where the drone reaches cruise altitude.
    # We do the same from the end for descent.

    def walk_path_forward(path: List[LatLon], target_dist: float
                          ) -> Tuple[LatLon, int, float]:
        """
        Walk forward along path until target_dist is covered.
        Returns (interpolated_point, segment_index_reached, overshoot).
        If path is shorter than target_dist, returns path[-1].
        """
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
        """Walk backward from path end. Returns (interpolated_point, index)."""
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

    # Cap climb/descent distances so they never exceed half the path
    max_one_side = total_path_len * 0.4
    climb_h_dist   = min(climb_h_dist,   max_one_side)
    descent_h_dist = min(descent_h_dist, max_one_side)

    climb_end_pt, climb_end_idx, _ = walk_path_forward(latlon_path, climb_h_dist)
    descent_start_pt, descent_start_idx = walk_path_backward(latlon_path, descent_h_dist)

    # If climb_end is past descent_start (very short route), use midpoint for both
    if climb_end_idx >= descent_start_idx:
        mid_lat = (latlon_path[0][0] + latlon_path[-1][0]) / 2
        mid_lon = (latlon_path[0][1] + latlon_path[-1][1]) / 2
        climb_end_pt     = (mid_lat, mid_lon)
        descent_start_pt = (mid_lat, mid_lon)
        climb_end_idx    = len(latlon_path) // 2
        descent_start_idx = climb_end_idx

    # ── Build list of (lat, lon, alt, phase) tuples ───────────────────────────
    segments: List[Tuple[float, float, float, FlightPhase]] = []

    pick_lat = pickup_pos.latitude
    pick_lon = pickup_pos.longitude
    deliv_lat = delivery_pos.latitude
    deliv_lon = delivery_pos.longitude

    # 1. Launch pad — ground
    segments.append((pick_lat, pick_lon, 0.0, FlightPhase.GROUND))

    # 2. Climb: linearly from z=0 to stratum, horizontally from pad to climb_end
    N_CLIMB = 4   # synthetic intermediate waypoints during climb
    for k in range(1, N_CLIMB + 1):
        t = k / N_CLIMB
        lat = pick_lat + t * (climb_end_pt[0] - pick_lat)
        lon = pick_lon + t * (climb_end_pt[1] - pick_lon)
        alt = t * stratum
        phase = (FlightPhase.TERMINAL
                 if _is_in_terminal_area(lat, lon, alt, pick_lat, pick_lon)
                 else FlightPhase.CLIMB)
        segments.append((lat, lon, alt, phase))

    # 3. Cruise: road waypoints between climb_end and descent_start
    cruise_pts = latlon_path[climb_end_idx+1 : descent_start_idx]
    for lat, lon in cruise_pts:
        segments.append((lat, lon, float(stratum), FlightPhase.CRUISE))

    # 4. Descent: from descent_start to delivery pad
    N_DESCENT = 4
    for k in range(1, N_DESCENT + 1):
        t = k / N_DESCENT
        lat = descent_start_pt[0] + t * (deliv_lat - descent_start_pt[0])
        lon = descent_start_pt[1] + t * (deliv_lon - descent_start_pt[1])
        alt = stratum * (1.0 - t)
        phase = (FlightPhase.TERMINAL
                 if _is_in_terminal_area(lat, lon, alt, deliv_lat, deliv_lon)
                 else FlightPhase.DESCENT)
        segments.append((lat, lon, alt, phase))

    # 5. Delivery pad — ground
    segments.append((deliv_lat, deliv_lon, 0.0, FlightPhase.GROUND))

    # ── Compute ETAs, sigma_t, and anchor bindings for every segment ──────────
    waypoints4d: List[Waypoint4D] = []
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
            sigma = _compute_sigma_t(0.0)
            wp_xyz = anchor_registry._to_xyz(lat, lon, alt)
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

        # 3-D distance for time calculation
        seg_3d = distance_3d(prev_lat, prev_lon, prev_alt, lat, lon, alt)
        seg_2d = haversine_distance(prev_lat, prev_lon, lat, lon)
        cumulative_arc += seg_2d
        total_dist     += seg_3d

        # Speed along segment depends on phase
        if phase in (FlightPhase.CLIMB, FlightPhase.TERMINAL) and alt > prev_alt:
            # Vertical component dominates — use 3D distance at climb rate
            vert_dist = alt - prev_alt
            seg_time  = vert_dist / climb_rate
        elif phase in (FlightPhase.DESCENT, FlightPhase.TERMINAL) and alt < prev_alt:
            vert_dist = prev_alt - alt
            seg_time  = vert_dist / descent_rate
        elif phase == FlightPhase.GROUND:
            seg_time = 0.5   # brief ground pause
        else:
            seg_time = seg_3d / cruise_speed

        current_time += seg_time
        sigma = _compute_sigma_t(cumulative_arc)
        heading = calculate_heading(prev_lat, prev_lon, lat, lon)

        speed = (climb_rate   if (phase == FlightPhase.CLIMB and alt > prev_alt)
                 else descent_rate if (phase == FlightPhase.DESCENT and alt < prev_alt)
                 else cruise_speed)

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
    """
    Return the straight line between start and goal.
    Road-unaware and zone-unaware — used only for reroute animation.
    """
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
    Plan a road-following, obstacle-aware 4-D trajectory.

    If stratum/direction are None, they are computed from the overall heading.
    Returns Trajectory4D (with climb/descent, sigma_t, phase tags),
    or None if no valid path could be found.
    """
    print(f"[Pathfinding] ({start.latitude:.4f},{start.longitude:.4f}) -> "
          f"({goal.latitude:.4f},{goal.longitude:.4f})")

    latlon_path: Optional[List[LatLon]] = None

    # ── Level 1: Direct OSRM ─────────────────────────────────
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
            print("[Pathfinding] Level 3: Visibility Graph A*")
            latlon_path = visibility_graph_astar(
                (start.latitude, start.longitude),
                (goal.latitude,  goal.longitude)
            )

    else:
        print("[Pathfinding] OSRM unavailable — Level 3 A*")
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
"""
4D Pathfinding Module
Implements A* algorithm with 4D nodes (lat, lon, alt, time)
Respects geofencing and optimizes for distance, energy, and time
"""

import heapq
import math
from typing import List, Tuple, Optional, Set
from models import Position, Waypoint, Trajectory
import config
import geofencing


class Node4D:
    """4D Node for A* pathfinding (Lat, Lon, Alt, Time)"""
    
    def __init__(self, lat: float, lon: float, alt: float, time: float, parent=None):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.time = time
        self.parent = parent
        self.g = 0.0  # Cost from start
        self.h = 0.0  # Heuristic to goal
        self.f = 0.0  # Total cost (g + h)
    
    def __lt__(self, other):
        return self.f < other.f
    
    def __eq__(self, other):
        return (abs(self.lat - other.lat) < 0.0001 and
                abs(self.lon - other.lon) < 0.0001 and
                abs(self.alt - other.alt) < 5.0)
    
    def __hash__(self):
        return hash((round(self.lat, 4), round(self.lon, 4), round(self.alt, 0)))
    
    def to_position(self) -> Position:
        return Position(latitude=self.lat, longitude=self.lon, altitude=self.alt)


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate great circle distance between two points on Earth
    
    Args:
        lat1, lon1: First point coordinates
        lat2, lon2: Second point coordinates
    
    Returns:
        Distance in meters
    """
    R = 6371000  # Earth radius in meters
    
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    a = (math.sin(delta_phi / 2) ** 2 +
         math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    return R * c


def distance_3d(lat1: float, lon1: float, alt1: float,
                lat2: float, lon2: float, alt2: float) -> float:
    """
    Calculate 3D Euclidean distance between two points
    
    Returns:
        Distance in meters
    """
    horizontal = haversine_distance(lat1, lon1, lat2, lon2)
    vertical = abs(alt2 - alt1)
    return math.sqrt(horizontal ** 2 + vertical ** 2)


def calculate_heading(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate heading (bearing) from point 1 to point 2
    
    Returns:
        Heading in degrees (0 = North, 90 = East)
    """
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    delta_lon = math.radians(lon2 - lon1)
    
    x = math.sin(delta_lon) * math.cos(lat2_rad)
    y = (math.cos(lat1_rad) * math.sin(lat2_rad) -
         math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon))
    
    heading = math.degrees(math.atan2(x, y))
    return (heading + 360) % 360


def get_neighbors(node: Node4D, goal_lat: float, goal_lon: float) -> List[Node4D]:
    """
    Generate neighboring nodes for A* expansion
    Considers horizontal movement, altitude changes, and time progression
    
    Args:
        node: Current node
        goal_lat, goal_lon: Goal coordinates for directional optimization
    
    Returns:
        List of valid neighbor nodes
    """
    neighbors = []
    
    # Calculate heading to goal for altitude stratification
    heading = calculate_heading(node.lat, node.lon, goal_lat, goal_lon)
    
    # Determine optimal altitude based on heading
    optimal_altitude = geofencing.get_safe_altitude_for_direction(heading, node.alt)
    
    # Movement options (in degrees, approximately 50m at equator)
    lat_step = config.GRID_RESOLUTION / 111320.0  # meters to degrees latitude
    lon_step = config.GRID_RESOLUTION / (111320.0 * math.cos(math.radians(node.lat)))
    
    # 8 horizontal directions + vertical movements
    directions = [
        (lat_step, 0),          # North
        (lat_step, lon_step),   # Northeast
        (0, lon_step),          # East
        (-lat_step, lon_step),  # Southeast
        (-lat_step, 0),         # South
        (-lat_step, -lon_step), # Southwest
        (0, -lon_step),         # West
        (lat_step, -lon_step),  # Northwest
    ]
    
    # Altitude options: stay, go to optimal, or intermediate steps
    altitude_options = [node.alt]
    if abs(optimal_altitude - node.alt) > 10:
        altitude_options.append(optimal_altitude)
        # Add intermediate altitude
        altitude_options.append((node.alt + optimal_altitude) / 2)
    
    for dlat, dlon in directions:
        for alt in altitude_options:
            new_lat = node.lat + dlat
            new_lon = node.lon + dlon
            
            # Check if within operational area
            if not geofencing.is_within_operational_area(
                Position(latitude=new_lat, longitude=new_lon, altitude=alt)
            ):
                continue
            
            # Calculate distance and time for this move
            dist = distance_3d(node.lat, node.lon, node.alt, new_lat, new_lon, alt)
            
            # Adjust speed based on altitude change (slower when climbing/descending)
            speed = config.DRONE_CRUISE_SPEED
            if abs(alt - node.alt) > 5:
                speed *= 0.8  # 20% slower when changing altitude
            
            time_increment = dist / speed
            new_time = node.time + time_increment
            
            neighbor = Node4D(new_lat, new_lon, alt, new_time, parent=node)
            neighbors.append(neighbor)
    
    return neighbors


def heuristic(node: Node4D, goal: Position) -> float:
    """
    Heuristic function for A* (admissible estimate to goal)
    Uses straight-line 3D distance divided by max speed
    
    Args:
        node: Current node
        goal: Goal position
    
    Returns:
        Estimated cost to goal
    """
    dist = distance_3d(node.lat, node.lon, node.alt,
                      goal.latitude, goal.longitude, goal.altitude)
    return dist / config.DRONE_MAX_SPEED


def reconstruct_path(node: Node4D) -> List[Node4D]:
    """
    Reconstruct path from goal node back to start
    
    Args:
        node: Goal node with parent chain
    
    Returns:
        List of nodes from start to goal
    """
    path = []
    current = node
    while current:
        path.append(current)
        current = current.parent
    return list(reversed(path))


def plan_trajectory(start: Position, goal: Position, start_time: float) -> Optional[Trajectory]:
    """
    Plan a 4D trajectory from start to goal using A* algorithm
    
    Args:
        start: Starting position
        goal: Goal position
        start_time: Mission start time (Unix timestamp)
    
    Returns:
        Trajectory object or None if no path found
    """
    # Initialize start and goal nodes
    # Use altitude stratification for initial altitude
    initial_heading = calculate_heading(start.latitude, start.longitude,
                                       goal.latitude, goal.longitude)
    start_altitude = geofencing.get_safe_altitude_for_direction(initial_heading, start.altitude)
    
    start_node = Node4D(start.latitude, start.longitude, start_altitude, start_time)
    goal_node = Node4D(goal.latitude, goal.longitude, goal.altitude, 0)
    
    # A* data structures
    open_set = []
    heapq.heappush(open_set, start_node)
    closed_set: Set[Node4D] = set()
    
    # Calculate heuristic for start
    start_node.h = heuristic(start_node, goal)
    start_node.f = start_node.h
    
    iterations = 0
    max_iterations = 200000  # Prevent infinite loops
    print("START:", start)
    print("GOAL:", goal)

    while open_set and iterations < max_iterations:
        iterations += 1
        current = heapq.heappop(open_set)
        
        # Check if we reached the goal
        goal_distance = distance_3d(current.lat, current.lon, current.alt,
                                    goal.latitude, goal.longitude, goal.altitude)
        if goal_distance < config.GRID_RESOLUTION*1.5:            # Reconstruct and convert path to trajectory
            path = reconstruct_path(current)
            return nodes_to_trajectory(path)
        
        closed_set.add(current)
        
        # Expand neighbors
        for neighbor in get_neighbors(current, goal.latitude, goal.longitude):
            
            if neighbor in closed_set:
                continue
            
            # Calculate movement cost
            move_cost = distance_3d(current.lat, current.lon, current.alt,
                                   neighbor.lat, neighbor.lon, neighbor.alt)
            
            # Apply geofencing cost multiplier
            position = neighbor.to_position()
            cost_multiplier = geofencing.get_position_cost_multiplier(position)
            
            if cost_multiplier == float('inf'):
                continue  # Skip prohibited areas
            
            move_cost *= cost_multiplier
            
            tentative_g = current.g + move_cost
            
            # Check if this path to neighbor is better
            neighbor.g = tentative_g
            neighbor.h = heuristic(neighbor, goal)
            neighbor.f = neighbor.g + neighbor.h
            neighbor.parent = current
            heapq.heappush(open_set, neighbor)
                
    print("A* failed after", iterations, "iterations")
    # No path found
    return None


def nodes_to_trajectory(nodes: List[Node4D]) -> Trajectory:
    """
    Convert list of 4D nodes to a Trajectory object
    
    Args:
        nodes: List of Node4D objects
    
    Returns:
        Trajectory with waypoints and metadata
    """
    waypoints = []
    total_distance = 0.0
    
    for i, node in enumerate(nodes):
        # Calculate speed and heading
        if i < len(nodes) - 1:
            next_node = nodes[i + 1]
            distance = distance_3d(node.lat, node.lon, node.alt,
                                  next_node.lat, next_node.lon, next_node.alt)
            time_diff = next_node.time - node.time
            speed = distance / time_diff if time_diff > 0 else config.DRONE_CRUISE_SPEED
            heading = calculate_heading(node.lat, node.lon, next_node.lat, next_node.lon)
            total_distance += distance
        else:
            speed = 0.0
            heading = 0.0
        
        waypoint = Waypoint(
            position=node.to_position(),
            eta=node.time,
            speed=min(speed, config.DRONE_MAX_SPEED),
            heading=heading
        )
        waypoints.append(waypoint)
    
    # Calculate battery usage (simplified model)
    total_time = nodes[-1].time - nodes[0].time
    battery_usage = (total_time * config.DRONE_POWER_CONSUMPTION / 
                    config.DRONE_BATTERY_CAPACITY * 100)
    
    return Trajectory(
        waypoints=waypoints,
        total_distance=total_distance,
        total_time=total_time,
        estimated_battery_usage=battery_usage
    )

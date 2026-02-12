"""
Geofencing Module
Handles no-fly zones, sensitive areas, and spatial cost calculations
Uses ray-casting algorithm for point-in-polygon tests
"""

from typing import List, Tuple
from models import Position
import config


def point_in_polygon(point: Tuple[float, float], polygon: List[Tuple[float, float]]) -> bool:
    """
    Ray-casting algorithm to determine if a point is inside a polygon
    
    Args:
        point: (latitude, longitude)
        polygon: List of (latitude, longitude) vertices
    
    Returns:
        True if point is inside polygon, False otherwise
    """
    lat, lon = point
    n = len(polygon)
    inside = False
    
    p1_lat, p1_lon = polygon[0]
    for i in range(1, n + 1):
        p2_lat, p2_lon = polygon[i % n]
        
        if lon > min(p1_lon, p2_lon):
            if lon <= max(p1_lon, p2_lon):
                if lat <= max(p1_lat, p2_lat):
                    if p1_lon != p2_lon:
                        xinters = (lon - p1_lon) * (p2_lat - p1_lat) / (p2_lon - p1_lon) + p1_lat
                    if p1_lat == p2_lat or lat <= xinters:
                        inside = not inside
        
        p1_lat, p1_lon = p2_lat, p2_lon
    
    return inside


def is_in_no_fly_zone(position: Position) -> bool:
    """
    Check if a position violates any no-fly zone
    
    Args:
        position: Position to check
    
    Returns:
        True if position is in a no-fly zone, False otherwise
    """
    point = (position.latitude, position.longitude)
    
    for zone in config.NO_FLY_ZONES:
        if point_in_polygon(point, zone['polygon']):
            return True
    
    return False


def get_position_cost_multiplier(position: Position) -> float:
    """
    Calculate the cost multiplier for a position based on geofencing
    
    Args:
        position: Position to evaluate
    
    Returns:
        Cost multiplier (1.0 = normal, >1.0 = discouraged, inf = prohibited)
    """
    point = (position.latitude, position.longitude)
    
    # Check no-fly zones first (infinite cost)
    for zone in config.NO_FLY_ZONES:
        if point_in_polygon(point, zone['polygon']):
            return float('inf')
    
    # Check sensitive areas (elevated cost)
    max_multiplier = 1.0
    for area in config.SENSITIVE_AREAS:
        if point_in_polygon(point, area['polygon']):
            max_multiplier = max(max_multiplier, area['cost_multiplier'])
    
    return max_multiplier


def is_trajectory_valid(waypoints: List[Position]) -> Tuple[bool, str]:
    """
    Validate that an entire trajectory respects geofencing constraints
    
    Args:
        waypoints: List of positions along the trajectory
    
    Returns:
        (is_valid, error_message)
    """
    for i, waypoint in enumerate(waypoints):
        if is_in_no_fly_zone(waypoint):
            return False, f"Waypoint {i} violates no-fly zone at ({waypoint.latitude:.4f}, {waypoint.longitude:.4f})"
    
    return True, "Trajectory is valid"


def get_safe_altitude_for_direction(heading: float, current_altitude: float) -> float:
    """
    Get the appropriate altitude based on heading direction (highway lanes)
    
    Args:
        heading: Direction in degrees (0 = North, 90 = East)
        current_altitude: Current altitude to potentially adjust
    
    Returns:
        Recommended altitude in meters
    """
    # Normalize heading to 0-360
    heading = heading % 360
    
    # Determine cardinal direction
    if 315 <= heading or heading < 45:
        direction = 'NORTH'
    elif 45 <= heading < 135:
        direction = 'EAST'
    elif 135 <= heading < 225:
        direction = 'SOUTH'
    else:
        direction = 'WEST'
    
    # Get available altitudes for this direction
    available_altitudes = config.DIRECTION_ALTITUDE_MAP.get(direction, config.ALTITUDE_LAYERS)
    
    # Find closest available altitude
    closest_altitude = min(available_altitudes, key=lambda alt: abs(alt - current_altitude))
    
    return closest_altitude


def is_within_operational_area(position: Position) -> bool:
    """
    Check if position is within the UTM system's operational boundaries
    
    Args:
        position: Position to check
    
    Returns:
        True if within bounds, False otherwise
    """
    bounds = config.OPERATIONAL_AREA
    
    if position.latitude < bounds['min_lat'] or position.latitude > bounds['max_lat']:
        return False
    
    if position.longitude < bounds['min_lon'] or position.longitude > bounds['max_lon']:
        return False
    
    return True


def get_geofence_info() -> dict:
    """
    Get information about all active geofences for visualization
    
    Returns:
        Dictionary containing all geofence zones
    """
    return {
        'no_fly_zones': config.NO_FLY_ZONES,
        'sensitive_areas': config.SENSITIVE_AREAS,
        'operational_area': config.OPERATIONAL_AREA
    }
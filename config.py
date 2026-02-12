"""
UTM System Configuration
Defines operational parameters, geofencing zones, and system constraints
"""

# ============================================================================
# OPERATIONAL PARAMETERS
# ============================================================================

# Drone performance characteristics
DRONE_MAX_SPEED = 15.0  # m/s (approximately 54 km/h)
DRONE_MIN_SPEED = 3.0   # m/s (minimum cruise speed)
DRONE_CRUISE_SPEED = 10.0  # m/s (optimal speed)
DRONE_MAX_ALTITUDE = 120.0  # meters (regulatory limit)
DRONE_MIN_ALTITUDE = 20.0   # meters (minimum safe altitude)
DRONE_BATTERY_CAPACITY = 3600.0  # Watt-hours
DRONE_POWER_CONSUMPTION = 200.0  # Watts at cruise

# Grid resolution for pathfinding
GRID_RESOLUTION = 100  # meters (horizontal grid spacing) 
ALTITUDE_LAYERS = [30, 50, 70, 90, 110]  # Discrete altitude levels in meters

# Conflict detection parameters
HORIZONTAL_SEPARATION = 30.0  # meters (minimum horizontal distance)
VERTICAL_SEPARATION = 15.0    # meters (minimum vertical distance)
TIME_RESOLUTION = 5.0          # seconds (temporal discretization)
LOOKAHEAD_TIME = 300.0         # seconds (5 minutes conflict prediction)

# ============================================================================
# ALTITUDE STRATIFICATION SYSTEM
# ============================================================================
# Highway lanes: Different altitudes for different cardinal directions
# This passively reduces collision probability

DIRECTION_ALTITUDE_MAP = {
    'NORTH': [50, 90],      # Northbound drones use these altitudes
    'SOUTH': [50, 90],      # Southbound drones use these altitudes
    'EAST': [30, 70, 110],  # Eastbound drones use these altitudes
    'WEST': [30, 70, 110],  # Westbound drones use these altitudes
}

# ============================================================================
# GEOFENCING - NO-FLY ZONES
# ============================================================================
# Defined as polygons (lat, lon) with infinite vertical extent
# Format: List of (latitude, longitude) tuples defining polygon vertices

NO_FLY_ZONES = [
    {
        'name': 'Airport Restricted Airspace',
        'polygon': [
            (37.6213, -122.3790),
            (37.6213, -122.3590),
            (37.6013, -122.3590),
            (37.6013, -122.3790),
        ],
        'cost_multiplier': float('inf')  # Absolute prohibition
    },
    {
        'name': 'Military Base',
        'polygon': [
            (37.7850, -122.4100),
            (37.7850, -122.3900),
            (37.7650, -122.3900),
            (37.7650, -122.4100),
        ],
        'cost_multiplier': float('inf')
    }
]

# ============================================================================
# SENSITIVE AREAS (High Cost, Not Prohibited)
# ============================================================================
# Areas where flight is discouraged but not forbidden
# These receive elevated costs in pathfinding to naturally route around them

SENSITIVE_AREAS = [
    {
        'name': 'Elementary School Zone',
        'polygon': [
            (37.7700, -122.4350),
            (37.7700, -122.4300),
            (37.7650, -122.4300),
            (37.7650, -122.4350),
        ],
        'cost_multiplier': 5.0  # 5x normal cost
    },
    {
        'name': 'Hospital Complex',
        'polygon': [
            (37.7550, -122.4050),
            (37.7550, -122.4000),
            (37.7500, -122.4000),
            (37.7500, -122.4050),
        ],
        'cost_multiplier': 4.0  # 4x normal cost
    },
    {
        'name': 'Residential High Density',
        'polygon': [
            (37.7400, -122.4200),
            (37.7400, -122.4100),
            (37.7300, -122.4100),
            (37.7300, -122.4200),
        ],
        'cost_multiplier': 2.0  # 2x normal cost
    }
]

# ============================================================================
# OPERATIONAL BOUNDS
# ============================================================================
# Geographic boundary for the UTM system

OPERATIONAL_AREA = {
    'min_lat': 37.60,
    'max_lat': 37.80,
    'min_lon': -122.45,
    'max_lon': -122.35
}

# ============================================================================
# SIMULATION PARAMETERS
# ============================================================================

# Virtual drone fleet size
SIMULATION_FLEET_SIZE = 10

# Delivery mission parameters
MISSION_INTERVAL = 30  # seconds between mission assignments
MISSION_TIMEOUT = 600   # seconds before mission is considered failed

# Update rates
TELEMETRY_UPDATE_RATE = 2.0  # Hz (updates per second)
CONFLICT_CHECK_INTERVAL = 5.0  # seconds

# ============================================================================
# API CONFIGURATION
# ============================================================================

API_HOST = "127.0.0.1"
API_PORT = 8000
CORS_ORIGINS = ["*"]  # Allow all origins for development

# WebSocket configuration
WS_HEARTBEAT_INTERVAL = 30  # seconds

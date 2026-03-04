"""
UTM System Configuration
"""

# ── Drone performance ─────────────────────────────────────────────────────────
DRONE_MAX_SPEED         = 50.0    # m/s
DRONE_MIN_SPEED         = 10.0    # m/s
DRONE_CRUISE_SPEED      = 35.0    # m/s
DRONE_MAX_ALTITUDE      = 130.0   # m  (top stratum ceiling)
DRONE_MIN_ALTITUDE      = 20.0    # m  (legacy, kept for geofencing)
DRONE_BATTERY_CAPACITY  = 3600.0  # Wh
DRONE_POWER_CONSUMPTION = 200.0   # W at cruise
DRONE_ACCEL_LIMIT  = 2.0   # m/s²  — max longitudinal acceleration / braking
DRONE_TURN_MAX_G   = 1.5   # g     — max lateral acceleration in a banked turn

# Kinematic constants — fixed for all drones for consistent CPA math
DRONE_CLIMB_RATE   = 3.0   # m/s vertical ascent
DRONE_DESCENT_RATE = 2.0   # m/s vertical 


# ── Altitude stratification (direction-exclusive) ─────────────────────────────
# Opposite-direction pairs are separated by ≥ 20 m, well above V_SEP=10 m.
# This provides structural conflict avoidance before any CPA logic runs.
#
#  WEST 30 m          EAST 50 m   (gap = 20 m)
#  SOUTH 40 m         NORTH 60 m  (gap = 20 m)
#  DIAGONAL primary 80 m, with fallbacks 100 m and 120 m
#
DIRECTION_STRATA = {
    'WEST':     30,
    'SOUTH':    40,
    'EAST':     50,
    'NORTH':    60,
    'DIAGONAL': 80,
}

# All available strata in ascending order.
# The resolver tries these when the assigned stratum is blocked.
ALL_STRATA_ORDERED = [30, 40, 50, 60, 80, 100, 120]

# ── CPA (Closest Point of Approach) parameters ────────────────────────────────
CPA_R_BASE   = 30.0   # m  — static horizontal protection radius (GPS floor)
CPA_SIGMA_0  = 2.0    # s  — irreducible timing uncertainty at launch
CPA_DRIFT_K  = 0.001  # s/m — uncertainty growth rate (1 s per 1 km)
CPA_V_SEP    = 10.0   # m  — minimum vertical separation

# ── Terminal area ─────────────────────────────────────────────────────────────
# Inside this cylinder around every pad, strata are suspended.
# A FIFO pad queue ensures sequential use of each pad.
TERMINAL_RADIUS       = 150.0   # m horizontal
TERMINAL_HEIGHT       = 50.0    # m vertical ceiling of terminal area
PAD_CLEARANCE_TIME    = 30.0    # s — gap between consecutive pad users

# ── Resolution cascade limits ─────────────────────────────────────────────────
MAX_TAKEOFF_DELAY       = 300.0  # s — reject beyond this
MAX_RESOLUTION_PASSES   = 5      # full re-validation loops
MAX_RESOLUTION_ATTEMPTS = 3      # per-conflict cascade attempts
RESOLUTION_TIMEOUT      = 8.0    # s — wall-clock planning limit
TEMPORAL_SAFETY_MARGIN  = 15.0   # s — added to combined sigma when computing delay

# ── Legacy grid (kept for A* pathfinding internals) ──────────────────────────
GRID_RESOLUTION  = 200
ALTITUDE_LAYERS  = [30, 40, 50, 60, 80, 100, 120]

# ── Geofencing ────────────────────────────────────────────────────────────────
NO_FLY_ZONES = [
    {
        'name': 'Airport Restricted Airspace',
        'polygon': [
            (37.6213, -122.3790), (37.6213, -122.3590),
            (37.6013, -122.3590), (37.6013, -122.3790),
        ],
        'cost_multiplier': 999999.0
    },
    {
        'name': 'Presidio Military Reserve',
        'polygon': [
            (37.8080, -122.5120), (37.8080, -122.4440),
            (37.7900, -122.4440), (37.7900, -122.5120),
        ],
        'cost_multiplier': 999999.0
    },
]

SENSITIVE_AREAS = [
    {
        'name': 'Elementary School Zone',
        'polygon': [
            (37.7700, -122.4350), (37.7700, -122.4300),
            (37.7650, -122.4300), (37.7650, -122.4350),
        ],
        'cost_multiplier': 5.0
    },
    {
        'name': 'Hospital Complex',
        'polygon': [
            (37.7550, -122.4050), (37.7550, -122.4000),
            (37.7500, -122.4000), (37.7500, -122.4050),
        ],
        'cost_multiplier': 4.0
    },
    {
        'name': 'Residential High Density',
        'polygon': [
            (37.7400, -122.4200), (37.7400, -122.4100),
            (37.7300, -122.4100), (37.7300, -122.4200),
        ],
        'cost_multiplier': 2.0
    },
]

OPERATIONAL_AREA = {
    'min_lat': 37.50, 'max_lat': 37.90,
    'min_lon': -122.55, 'max_lon': -122.25
}

# ── API ───────────────────────────────────────────────────────────────────────
API_HOST  = "0.0.0.0"
API_PORT  = 8000
CORS_ORIGINS = ["*"]

SIMULATION_FLEET_SIZE    = 10
MISSION_INTERVAL         = 30
MISSION_TIMEOUT          = 600
TELEMETRY_UPDATE_RATE    = 2.0
CONFLICT_CHECK_INTERVAL  = 5.0
WS_HEARTBEAT_INTERVAL    = 30

# ── GPS-Denied / UWB Ranging ──────────────────────────────────────────────────
# Noise model:  σᵢ = UWB_SIGMA_0 + UWB_DRIFT_K * d̂ᵢ
# Represents physical UWB radio ranging uncertainty growing with distance.
UWB_SIGMA_0       = 0.5     # m — constant floor noise (GPS-denied)
UWB_DRIFT_K       = 0.005   # m/m — distance-proportional noise (0.5% of range)

# Anchor selection per waypoint
ANCHOR_MAX_RANGE  = 2500.0  # m — ranging radius; 2.5 km suits the 25-node SF grid
ANCHOR_MIN_COUNT  = 4       # minimum nodes required for a valid WLS fix
ANCHOR_MAX_COUNT  = 6       # maximum nodes used per waypoint (GDOP-optimised)

# GDOP thresholds — split into horizontal and vertical components
HDOP_THRESHOLD    = 3.0     # above this: fall back to dead reckoning (horizontal)
VDOP_THRESHOLD    = 3.0     # above this: zero out vertical correction (use baro)

# WLS correction parameters
WLS_CORRECTION_TAU = 3.0    # s — time window to apply velocity correction

# ── Legacy ────────────────────────────────────────────────────────────────────
HORIZONTAL_SEPARATION = CPA_R_BASE
VERTICAL_SEPARATION   = CPA_V_SEP
TIME_RESOLUTION       = 5.0
LOOKAHEAD_TIME        = 300.0
DIRECTION_ALTITUDE_MAP = {
    'NORTH': [60], 'SOUTH': [40], 'EAST': [50], 'WEST': [30],
}
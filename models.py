"""
Data Models for UTM System
"""

from pydantic import BaseModel, Field
from typing import List, Optional, Tuple
from enum import Enum


class DroneStatus(str, Enum):
    IDLE              = "idle"
    ASSIGNED          = "assigned"
    EN_ROUTE_PICKUP   = "en_route_pickup"
    AT_PICKUP         = "at_pickup"
    EN_ROUTE_DELIVERY = "en_route_delivery"
    AT_DELIVERY       = "at_delivery"
    RETURNING         = "returning"
    EMERGENCY         = "emergency"
    MAINTENANCE       = "maintenance"


class FlightPhase(str, Enum):
    """Phase tag on every Waypoint4D.

    GROUND   - on the pad (z=0), before or after flight
    TERMINAL - inside the terminal cylinder (r<150m from pad, alt<50m)
               strata rules suspended; FIFO pad-queue governs
    CLIMB    - ascending from ground to cruise stratum
    CRUISE   - level flight at assigned stratum altitude
    DESCENT  - descending from cruise stratum to ground
    """
    GROUND   = "ground"
    TERMINAL = "terminal"
    CLIMB    = "climb"
    CRUISE   = "cruise"
    DESCENT  = "descent"


class Position(BaseModel):
    latitude:  float = Field(..., ge=-90,  le=90)
    longitude: float = Field(..., ge=-180, le=180)
    altitude:  float = Field(..., ge=0,    le=150)


class Position4D(Position):
    timestamp: float


class Waypoint(BaseModel):
    """Legacy waypoint - kept for frontend JSON compatibility."""
    position: Position
    eta:      float
    speed:    float
    heading:  float


class AnchorBinding(BaseModel):
    """
    Pre-computed distance binding from a waypoint to one infrastructure node.

    Stored in Waypoint4D.anchor_bindings at planning time.
    Used at execution time to compute WLS position correction.

    node_id          -- matches InfrastructureNode.node_id in the anchor registry
    expected_dist_m  -- 3-D Euclidean distance (metres) at plan time
    """
    node_id:          str
    expected_dist_m:  float


class Waypoint4D(BaseModel):
    """
    Full 4-D waypoint with uncertainty, phase annotation, and anchor bindings.

    sigma_t  -- timing uncertainty at this waypoint (seconds).
                Grows linearly with arc-length from launch:
                  sigma_t = SIGMA_0 + DRIFT_K * cumulative_arc_metres
                Used by CPA to compute effective protection radius:
                  R_eff = R_BASE + speed * sigma_t

    phase    -- FlightPhase tag; drives which conflict-check rules apply.

    anchor_bindings -- 4-6 pre-selected infrastructure nodes with expected
                       distances for GPS-denied WLS navigation correction.
                       Empty list means no anchor coverage (CRUISE-only strata
                       far from nodes); drone falls back to dead reckoning.
    """
    position:        Position
    eta:             float
    speed:           float
    heading:         float
    phase:           FlightPhase    = FlightPhase.CRUISE
    sigma_t:         float          = 2.0
    anchor_bindings: List[AnchorBinding] = []


class Trajectory(BaseModel):
    """Legacy trajectory - used by the frontend SampledPositionProperty."""
    waypoints:               List[Waypoint]
    total_distance:          float
    total_time:              float
    estimated_battery_usage: float


class Trajectory4D(BaseModel):
    """
    Full 4-D trajectory used internally by the conflict-resolution pipeline.

    takeoff_delay   -- seconds added at planning time (default 0).
    base_start_time -- original requested launch time (before any delay).
    stratum         -- assigned cruise altitude in metres.
    direction       -- cardinal direction for stratum assignment.
    """
    waypoints:               List[Waypoint4D]
    total_distance:          float
    total_time:              float
    estimated_battery_usage: float
    takeoff_delay:           float = 0.0
    base_start_time:         float = 0.0
    stratum:                 int   = 50
    direction:               str   = "NORTH"

    def to_legacy(self) -> Trajectory:
        """Convert to legacy Trajectory for frontend broadcast."""
        return Trajectory(
            waypoints=[
                Waypoint(
                    position=wp.position,
                    eta=wp.eta,
                    speed=wp.speed,
                    heading=wp.heading,
                )
                for wp in self.waypoints
            ],
            total_distance=self.total_distance,
            total_time=self.total_time,
            estimated_battery_usage=self.estimated_battery_usage,
        )


class DeliveryRequest(BaseModel):
    pickup:   Position
    delivery: Position


class Mission(BaseModel):
    mission_id:        str
    drone_id:          Optional[str]       = None
    pickup_location:   Position
    delivery_location: Position
    created_at:        float
    assigned_at:       Optional[float]     = None
    completed_at:      Optional[float]     = None
    status:            DroneStatus         = DroneStatus.IDLE
    trajectory:        Optional[Trajectory] = None


class ConflictAlert(BaseModel):
    conflict_id:       str
    drone_1_id:        str
    drone_2_id:        str
    conflict_position: Position
    conflict_time:     float
    severity:          str
    resolution_action: Optional[str] = None


class Telemetry(BaseModel):
    drone_id:      str
    position:      Position
    velocity:      Tuple[float, float, float]
    battery_level: float
    status:        DroneStatus
    timestamp:     float


class DroneRegistration(BaseModel):
    drone_id:     str
    model:        str   = "DJI_Delivery_X1"
    max_payload:  float = 5.0
    max_range:    float = 15000.0
    cruise_speed: float = 10.0


class FlightPlan(BaseModel):
    plan_id:            str
    drone_id:           str
    trajectory:         Trajectory
    approved:           bool               = False
    approval_time:      Optional[float]    = None
    conflicts_detected: List[ConflictAlert] = []


class GeofenceZone(BaseModel):
    zone_id:         str
    name:            str
    polygon:         List[Tuple[float, float]]
    altitude_min:    Optional[float] = None
    altitude_max:    Optional[float] = None
    cost_multiplier: float           = 1.0
    active:          bool            = True


class SystemStatus(BaseModel):
    active_drones:       int
    active_missions:     int
    total_flights_today: int
    conflicts_detected:  int
    conflicts_resolved:  int
    system_health:       str
    timestamp:           float
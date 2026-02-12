"""
Data Models for UTM System
Uses Pydantic for validation and serialization
"""

from pydantic import BaseModel, Field
from typing import List, Optional, Tuple
from datetime import datetime
from enum import Enum


class DroneStatus(str, Enum):
    """Drone operational states"""
    IDLE = "idle"
    ASSIGNED = "assigned"
    EN_ROUTE_PICKUP = "en_route_pickup"
    AT_PICKUP = "at_pickup"
    EN_ROUTE_DELIVERY = "en_route_delivery"
    AT_DELIVERY = "at_delivery"
    RETURNING = "returning"
    EMERGENCY = "emergency"
    MAINTENANCE = "maintenance"


class Position(BaseModel):
    """3D Position (Lat, Lon, Alt)"""
    latitude: float = Field(..., ge=-90, le=90)
    longitude: float = Field(..., ge=-180, le=180)
    altitude: float = Field(..., ge=0, le=150)  # meters above ground


class Position4D(Position):
    """4D Position (Lat, Lon, Alt, Time)"""
    timestamp: float  # Unix timestamp


class Waypoint(BaseModel):
    """Single waypoint in a trajectory"""
    position: Position
    eta: float  # Estimated Time of Arrival (Unix timestamp)
    speed: float  # Target speed in m/s
    heading: float  # Degrees (0 = North, 90 = East)


class Trajectory(BaseModel):
    """Complete flight path"""
    waypoints: List[Waypoint]
    total_distance: float  # meters
    total_time: float  # seconds
    estimated_battery_usage: float  # percentage


class Telemetry(BaseModel):
    """Real-time drone telemetry data"""
    drone_id: str
    position: Position
    velocity: Tuple[float, float, float]  # (vx, vy, vz) in m/s
    battery_level: float  # percentage (0-100)
    status: DroneStatus
    timestamp: float  # Unix timestamp


class Mission(BaseModel):
    """Delivery mission definition"""
    mission_id: str
    drone_id: Optional[str] = None
    pickup_location: Position
    delivery_location: Position
    created_at: float
    assigned_at: Optional[float] = None
    completed_at: Optional[float] = None
    status: DroneStatus = DroneStatus.IDLE
    trajectory: Optional[Trajectory] = None


class DeliveryRequest(BaseModel):
    """User request to create a delivery"""
    pickup: Position
    delivery: Position


class ConflictAlert(BaseModel):
    """Conflict detection result"""
    conflict_id: str
    drone_1_id: str
    drone_2_id: str
    conflict_position: Position
    conflict_time: float  # Unix timestamp
    severity: str  # "critical", "warning", "resolved"
    resolution_action: Optional[str] = None


class DroneRegistration(BaseModel):
    """Drone registration data"""
    drone_id: str
    model: str = "DJI_Delivery_X1"
    max_payload: float = 5.0  # kg
    max_range: float = 15000.0  # meters
    cruise_speed: float = 10.0  # m/s


class FlightPlan(BaseModel):
    """Approved flight plan"""
    plan_id: str
    drone_id: str
    trajectory: Trajectory
    approved: bool = False
    approval_time: Optional[float] = None
    conflicts_detected: List[ConflictAlert] = []


class GeofenceZone(BaseModel):
    """Geofenced area definition"""
    zone_id: str
    name: str
    polygon: List[Tuple[float, float]]  # List of (lat, lon) vertices
    altitude_min: Optional[float] = None
    altitude_max: Optional[float] = None
    cost_multiplier: float = 1.0
    active: bool = True


class SystemStatus(BaseModel):
    """Overall UTM system status"""
    active_drones: int
    active_missions: int
    total_flights_today: int
    conflicts_detected: int
    conflicts_resolved: int
    system_health: str  # "operational", "degraded", "offline"
    timestamp: float

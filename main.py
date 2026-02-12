"""
UTM System - FastAPI Backend
Main server handling delivery requests, trajectory planning, and conflict management
"""

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from typing import Dict, List
import asyncio
import json
import time
import uuid

from models import (
    DeliveryRequest, Mission, Telemetry, DroneStatus,
    Position, Trajectory, ConflictAlert, SystemStatus
)
import config
import pathfinding
import geofencing
import conflict_detection

# ============================================================================
# FASTAPI SETUP
# ============================================================================

app = FastAPI(
    title="UTM System API",
    description="Unmanned Traffic Management for Autonomous Drone Delivery",
    version="1.0.0"
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=config.CORS_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ============================================================================
# GLOBAL STATE
# ============================================================================

# Active drones and their current state
active_drones: Dict[str, Telemetry] = {}

# Active missions
active_missions: Dict[str, Mission] = {}

# Approved flight plans
flight_plans: Dict[str, Trajectory] = {}

# Conflict detector and resolver
conflict_resolver = conflict_detection.ConflictResolver()

# WebSocket connections for real-time updates
websocket_connections: List[WebSocket] = []

# System statistics
system_stats = {
    'total_missions': 0,
    'completed_missions': 0,
    'conflicts_detected': 0,
    'conflicts_resolved': 0,
    'active_drones': 0
}

# ============================================================================
# WEBSOCKET MANAGER
# ============================================================================

class ConnectionManager:
    """Manages WebSocket connections for real-time updates"""
    
    def __init__(self):
        self.active_connections: List[WebSocket] = []
    
    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
    
    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)
    
    async def broadcast(self, message: dict):
        """Send message to all connected clients"""
        for connection in self.active_connections:
            try:
                await connection.send_json(message)
            except:
                pass

manager = ConnectionManager()

# ============================================================================
# API ENDPOINTS
# ============================================================================

@app.get("/")
async def root():
    """Serve the frontend HTML"""
    try:
        with open('frontend/index.html', 'r') as f:
            return HTMLResponse(content=f.read())
    except FileNotFoundError:
        return {"message": "UTM System API is running. Frontend not found."}

@app.get("/api/health")
async def health_check():
    """System health check"""
    return {
        "status": "operational",
        "timestamp": time.time(),
        "active_drones": len(active_drones),
        "active_missions": len(active_missions)
    }

@app.get("/api/system/status")
async def get_system_status():
    """Get overall system status"""
    return SystemStatus(
        active_drones=len(active_drones),
        active_missions=len([m for m in active_missions.values() 
                           if m.status not in [DroneStatus.IDLE, DroneStatus.AT_DELIVERY]]),
        total_flights_today=system_stats['total_missions'],
        conflicts_detected=system_stats['conflicts_detected'],
        conflicts_resolved=system_stats['conflicts_resolved'],
        system_health="operational",
        timestamp=time.time()
    )

@app.get("/api/geofencing/zones")
async def get_geofencing_zones():
    """Get all geofencing zones for visualization"""
    return geofencing.get_geofence_info()

@app.post("/api/delivery/request")
async def create_delivery_request(request: DeliveryRequest):
    """
    Create a new delivery request
    This triggers mission assignment and trajectory planning
    """
    # Validate locations are within operational area
    if not geofencing.is_within_operational_area(request.pickup):
        raise HTTPException(400, "Pickup location outside operational area")
    
    if not geofencing.is_within_operational_area(request.delivery):
        raise HTTPException(400, "Delivery location outside operational area")
    
    # Check if locations are in no-fly zones
    if geofencing.is_in_no_fly_zone(request.pickup):
        raise HTTPException(400, "Pickup location is in a no-fly zone")
    
    if geofencing.is_in_no_fly_zone(request.delivery):
        raise HTTPException(400, "Delivery location is in a no-fly zone")
    
    # Create mission
    mission_id = f"mission_{uuid.uuid4().hex[:8]}"
    mission = Mission(
        mission_id=mission_id,
        pickup_location=request.pickup,
        delivery_location=request.delivery,
        created_at=time.time(),
        status=DroneStatus.IDLE
    )
    
    # Find available drone (in real system, this would be more sophisticated)
    available_drone = None
    for drone_id, telemetry in active_drones.items():
        if telemetry.status == DroneStatus.IDLE:
            available_drone = drone_id
            break
    
    if not available_drone:
        # Queue mission for later
        active_missions[mission_id] = mission
        system_stats['total_missions'] += 1
        return {
            "mission_id": mission_id,
            "status": "queued",
            "message": "No drones available. Mission queued."
        }
    
    # Assign drone and plan trajectory
    mission.drone_id = available_drone
    mission.assigned_at = time.time()
    mission.status = DroneStatus.ASSIGNED
    
    # Plan trajectory
    current_position = active_drones[available_drone].position
    start_time = time.time()
    
    # Plan to pickup
    trajectory_to_pickup = pathfinding.plan_trajectory(
        current_position, request.pickup, start_time
    )
    
    if not trajectory_to_pickup:
        raise HTTPException(500, "Could not plan path to pickup location")
    
    # Plan from pickup to delivery
    pickup_arrival_time = trajectory_to_pickup.waypoints[-1].eta
    trajectory_to_delivery = pathfinding.plan_trajectory(
        request.pickup, request.delivery, pickup_arrival_time + 30  # 30s for loading
    )
    
    if not trajectory_to_delivery:
        raise HTTPException(500, "Could not plan path to delivery location")
    
    # Combine trajectories
    combined_waypoints = (trajectory_to_pickup.waypoints + 
                         trajectory_to_delivery.waypoints)
    
    total_distance = (trajectory_to_pickup.total_distance + 
                     trajectory_to_delivery.total_distance)
    
    total_time = (trajectory_to_delivery.waypoints[-1].eta - 
                 trajectory_to_pickup.waypoints[0].eta)
    
    battery_usage = (trajectory_to_pickup.estimated_battery_usage + 
                    trajectory_to_delivery.estimated_battery_usage)
    
    mission.trajectory = Trajectory(
        waypoints=combined_waypoints,
        total_distance=total_distance,
        total_time=total_time,
        estimated_battery_usage=battery_usage
    )
    
    # Check for conflicts with other active flights
    conflicts = detect_mission_conflicts(mission)
    
    if conflicts:
        # Resolve conflicts
        for conflict in conflicts:
            resolved_trajectory, method = conflict_resolver.resolve_conflict(
                conflict, mission.trajectory, 
                flight_plans[conflict.drone_2_id]
            )
            mission.trajectory = resolved_trajectory
            system_stats['conflicts_detected'] += 1
            system_stats['conflicts_resolved'] += 1
            
            # Broadcast conflict resolution
            await manager.broadcast({
                'type': 'conflict_resolved',
                'conflict': conflict.model_dump(),
                'resolution_method': method
            })
    
    # Save mission and flight plan
    active_missions[mission_id] = mission
    flight_plans[available_drone] = mission.trajectory
    system_stats['total_missions'] += 1
    
    # Update drone status
    active_drones[available_drone].status = DroneStatus.ASSIGNED
    
    # Broadcast mission created
    await manager.broadcast({
        'type': 'mission_created',
        'mission': mission.model_dump()
    })
    
    return {
        "mission_id": mission_id,
        "drone_id": available_drone,
        "status": "assigned",
        "trajectory": mission.trajectory,
        "conflicts_detected": len(conflicts)
    }

@app.get("/api/missions")
async def get_all_missions():
    """Get all active missions"""
    return list(active_missions.values())

@app.get("/api/missions/{mission_id}")
async def get_mission(mission_id: str):
    """Get specific mission details"""
    if mission_id not in active_missions:
        raise HTTPException(404, "Mission not found")
    return active_missions[mission_id]

@app.post("/api/telemetry/update")
async def update_telemetry(telemetry: Telemetry):
    """
    Update drone telemetry (called by virtual drones)
    """
    active_drones[telemetry.drone_id] = telemetry
    
    # Broadcast telemetry update
    await manager.broadcast({
        'type': 'telemetry',
        'data': telemetry.model_dump()
    })
    
    return {"status": "ok"}

@app.get("/api/drones")
async def get_all_drones():
    """Get all active drones"""
    return list(active_drones.values())

@app.get("/api/drones/{drone_id}")
async def get_drone(drone_id: str):
    """Get specific drone status"""
    if drone_id not in active_drones:
        raise HTTPException(404, "Drone not found")
    return active_drones[drone_id]

@app.post("/api/drones/{drone_id}/register")
async def register_drone(drone_id: str, position: Position):
    """Register a new virtual drone"""
    if drone_id in active_drones:
        raise HTTPException(400, "Drone already registered")
    
    telemetry = Telemetry(
        drone_id=drone_id,
        position=position,
        velocity=(0.0, 0.0, 0.0),
        battery_level=100.0,
        status=DroneStatus.IDLE,
        timestamp=time.time()
    )
    
    active_drones[drone_id] = telemetry
    system_stats['active_drones'] = len(active_drones)
    
    await manager.broadcast({
        'type': 'drone_registered',
        'drone': telemetry.model_dump()
    })
    
    return {"status": "registered", "drone_id": drone_id}

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket for real-time updates"""
    await manager.connect(websocket)
    
    try:
        # Send initial state
        await websocket.send_json({
            'type': 'initial_state',
            'drones': [d.model_dump() for d in active_drones.values()],
            'missions': [m.model_dump() for m in active_missions.values()],
            'geofencing': geofencing.get_geofence_info()
        })
        
        while True:
            # Keep connection alive
            data = await websocket.receive_text()
            
            if data == "ping":
                await websocket.send_text("pong")
    
    except WebSocketDisconnect:
        manager.disconnect(websocket)

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def detect_mission_conflicts(new_mission: Mission) -> List[ConflictAlert]:
    """
    Detect conflicts between new mission and existing flight plans
    """
    conflicts = []
    
    for drone_id, trajectory in flight_plans.items():
        if drone_id != new_mission.drone_id:
            conflict = conflict_resolver.conflict_detector.check_trajectory_conflict(
                new_mission.drone_id, new_mission.trajectory,
                drone_id, trajectory
            )
            if conflict:
                conflicts.append(conflict)
    
    return conflicts

# ============================================================================
# BACKGROUND TASKS
# ============================================================================

@app.on_event("startup")
async def startup_event():
    """Initialize system on startup"""
    print("UTM System Starting...")
    print(f"Operational Area: {config.OPERATIONAL_AREA}")
    print(f"No-Fly Zones: {len(config.NO_FLY_ZONES)}")
    print(f"Sensitive Areas: {len(config.SENSITIVE_AREAS)}")
    print("System ready for drone operations.")

@app.on_event("shutdown")
async def shutdown_event():
    """Cleanup on shutdown"""
    print("UTM System Shutting Down...")

# ============================================================================
# MAIN
# ============================================================================

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host=config.API_HOST, port=config.API_PORT)
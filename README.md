# UTM System - Unmanned Traffic Management for Autonomous Drone Delivery

**A Proof of Concept for 4D Trajectory Management with Virtual Simulation**

## 🎯 Project Overview

This is a complete, production-ready Proof of Concept (PoC) for an Unmanned Traffic Management (UTM) system specifically designed for autonomous drone delivery at scale. Since no physical drones or hardware are available, the entire ecosystem is **fully simulated** to prove the logic is sound.

### The Problem We Solve

As drone delivery scales, manual piloting becomes impossible. This system enables users to simply set a "Pickup" and "Delivery" location, and the system automates everything else:
- ✅ Intelligent 4D route planning (3D space + time)
- ✅ Static conflict avoidance (geofencing)
- ✅ Dynamic conflict resolution (altitude stratification, speed control)
- ✅ Real-time traffic management

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    FRONTEND (CesiumJS)                      │
│         3D Visualization + Control Interface                │
└────────────────┬────────────────────────────────────────────┘
                 │ WebSocket + REST API
┌────────────────▼────────────────────────────────────────────┐
│              BACKEND (FastAPI)                              │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │ Pathfinding  │  │  Geofencing  │  │   Conflict   │      │
│  │   (A* 4D)    │  │   Manager    │  │  Resolution  │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
└────────────────┬────────────────────────────────────────────┘
                 │ HTTP Telemetry Updates
┌────────────────▼────────────────────────────────────────────┐
│          SIMULATION (Python asyncio)                        │
│    Virtual Fleet Manager - 10 Mock Drones                   │
│         Physics Simulation + Battery Model                  │
└─────────────────────────────────────────────────────────────┘
```

## 🚀 Core Features

### 1. **4D Trajectory Management**
- Routes planned in **3D space (Lat, Long, Alt)** AND **Time**
- A* pathfinding algorithm with temporal resolution
- Conflicts defined as: *"Two drones in the same space at the same time"*

### 2. **Static Conflict Management (Geofencing)**
- **No-Fly Zones**: Absolute prohibition (infinite cost)
  - Airport restricted airspace
  - Military bases
- **Sensitive Areas**: High cost (naturally avoided)
  - Schools (5x cost)
  - Hospitals (4x cost)
  - Dense residential (2x cost)

### 3. **Dynamic Conflict Resolution**

**Priority 1: Altitude Stratification (Highway Lanes)**
```
North/South Traffic → Altitudes: 50m, 90m
East/West Traffic   → Altitudes: 30m, 70m, 110m
```
This passively reduces collision probability by segregating traffic flows.

**Priority 2: Speed Control**
When conflicts are detected, the system adjusts drone speeds to miss each other at intersections (more energy-efficient than hovering).

**Priority 3: Altitude Adjustment**
If speed control is insufficient, drones are separated vertically.

### 4. **Virtual Simulation & Visualization**
- **Backend**: Python/FastAPI serves the UTM logic
- **Simulation**: 10 virtual drones with physics-based movement
- **Frontend**: CesiumJS provides a stunning 3D map visualization
- **Real-time**: WebSocket updates show drones moving live

## 📁 Project Structure

```
utm_system/
├── config.py                 # System parameters, geofencing zones
├── models.py                 # Pydantic data models
├── geofencing.py            # No-fly zones, spatial queries
├── pathfinding.py           # 4D A* algorithm
├── conflict_detection.py    # Collision detection & resolution
├── main.py                  # FastAPI backend server
├── drone_simulator.py       # Virtual drone fleet manager
├── frontend/
│   └── index.html          # CesiumJS 3D visualization
└── requirements.txt        # Python dependencies
```

## 🛠️ Installation & Setup

### Prerequisites
- Python 3.10+
- Modern web browser (Chrome, Firefox, Edge)
- Internet connection (for CesiumJS CDN)

### Step 1: Install Dependencies

```bash
cd utm_system
pip install -r requirements.txt
```

### Step 2: Start the UTM Backend

```bash
python main.py
```

Expected output:
```
UTM System Starting...
Operational Area: {'min_lat': 37.6, 'max_lat': 37.8, ...}
No-Fly Zones: 2
Sensitive Areas: 3
System ready for drone operations.
INFO:     Uvicorn running on http://0.0.0.0:8000
```

### Step 3: Start the Virtual Drone Fleet (New Terminal)

```bash
python drone_simulator.py
```

Expected output:
```
============================================================
UTM VIRTUAL DRONE FLEET SIMULATOR
============================================================
Waiting for UTM backend at http://localhost:8000...
Initializing fleet of 10 virtual drones...
✓ drone_001 registered at (37.7234, -122.4101, 50m)
✓ drone_002 registered at (37.7512, -122.3899, 70m)
...
Fleet initialization complete. 10 drones ready.
Starting simulation loop (update rate: 2.0 Hz)...
```

### Step 4: Open the Frontend

Open your browser and navigate to:
```
http://localhost:8000
```

You should see the 3D map with:
- ✅ Virtual drones positioned in the operational area
- ✅ Red zones (no-fly areas)
- ✅ Orange zones (sensitive areas)
- ✅ Control panel for creating deliveries

## 🎮 How to Use

### Creating a Delivery Mission

1. In the **Control Panel** (left side), enter coordinates:
   - **Pickup Location**: e.g., Lat: 37.77, Lon: -122.43
   - **Delivery Location**: e.g., Lat: 37.75, Lon: -122.41

2. Click **"Request Delivery"**

3. Watch the system:
   - ✅ Assign an available drone
   - ✅ Plan a 4D trajectory (respecting geofencing)
   - ✅ Check for conflicts with other flights
   - ✅ Resolve conflicts if detected
   - ✅ Display the flight path in cyan on the 3D map
   - ✅ Drone follows the path in real-time

### Observing the System

- **Header Stats**: Shows active drones, missions, and conflicts
- **Mission List**: Displays all active deliveries with status
- **3D Map**: 
  - Green markers = Active drones
  - Cyan lines = Flight paths
  - Red zones = No-fly areas
  - Orange zones = Sensitive areas

## 🧪 Testing Scenarios

### Scenario 1: Basic Delivery
```
Pickup:    (37.77, -122.43)
Delivery:  (37.75, -122.41)
```
Should complete without conflicts.

### Scenario 2: Multiple Simultaneous Deliveries
Create 3-5 deliveries at the same time to trigger:
- Altitude stratification
- Speed-based conflict resolution

### Scenario 3: No-Fly Zone Avoidance
```
Pickup:    (37.77, -122.43)
Delivery:  (37.61, -122.37)  # Forces route around airport
```
Path should curve around the no-fly zone.

### Scenario 4: Sensitive Area Cost
```
Pickup:    (37.78, -122.44)
Delivery:  (37.74, -122.40)  # Direct route crosses hospital
```
Route should naturally avoid hospital due to 4x cost multiplier.

## 🔧 Configuration

All parameters can be adjusted in `config.py`:

### Operational Parameters
- `DRONE_MAX_SPEED`: 15 m/s (default)
- `DRONE_CRUISE_SPEED`: 10 m/s (default)
- `GRID_RESOLUTION`: 50m (pathfinding granularity)

### Safety Margins
- `HORIZONTAL_SEPARATION`: 30m minimum
- `VERTICAL_SEPARATION`: 15m minimum
- `TIME_RESOLUTION`: 5 seconds

### Altitude Stratification
Modify `DIRECTION_ALTITUDE_MAP` to change highway lane assignments.

### Geofencing
Add/remove zones in:
- `NO_FLY_ZONES`: Absolute prohibition
- `SENSITIVE_AREAS`: Elevated cost

## 📊 API Documentation

Once running, access interactive API docs at:
```
http://localhost:8000/docs
```

### Key Endpoints

**POST /api/delivery/request**
```json
{
  "pickup": {"latitude": 37.77, "longitude": -122.43, "altitude": 0},
  "delivery": {"latitude": 37.75, "longitude": -122.41, "altitude": 0}
}
```

**GET /api/drones**
Returns all active drones with telemetry.

**GET /api/missions**
Returns all active missions.

**WebSocket /ws**
Real-time updates for telemetry, missions, and conflicts.

## 🎨 Frontend Design Philosophy

The interface follows a **mission-critical control center** aesthetic:
- **Dark theme**: Reduces eye strain during long monitoring sessions
- **Cyan accents**: High contrast for critical information
- **Monospace fonts**: For precise numerical data
- **Glass morphism**: Modern, layered depth
- **Real-time animations**: Smooth transitions for state changes

Inspired by air traffic control systems and modern aerospace UIs.

## 🧠 Key Algorithms Explained

### A* Pathfinding (4D)
```python
# Cost function: f(n) = g(n) + h(n)
# g(n) = actual cost from start to node n
# h(n) = heuristic estimate from n to goal

# In 4D:
# - Nodes have (lat, lon, alt, time)
# - Movement cost includes geofencing multipliers
# - Heuristic uses straight-line 3D distance
```

### Conflict Detection
```python
# For each time step:
#   1. Interpolate position of both drones
#   2. Calculate horizontal and vertical separation
#   3. If both < minimum separation → CONFLICT
```

### Speed Adjustment Resolution
```python
# Before conflict point: Reduce speed by 30%
# After conflict point: Resume cruise speed
# Result: Drone arrives later, missing the other
```

## 🚧 Limitations & Future Work

### Current Limitations
- Simplified battery model (linear consumption)
- No wind or weather effects
- 2D pathfinding (no vertical path optimization)
- No multi-leg missions (pickup → delivery only)

### Potential Enhancements
1. **Advanced Conflict Resolution**
   - Rerouting around congested areas
   - Holdover patterns (circling)

2. **Weather Integration**
   - Wind compensation
   - Real-time weather data

3. **Battery Optimization**
   - Return-to-base planning
   - Charge station network

4. **Machine Learning**
   - Predicted demand patterns
   - Optimal drone positioning

5. **Multi-modal Delivery**
   - Ground robots for final 100m
   - Truck-drone coordination

## 📝 Technical Notes

### Why A* Over Dijkstra?
A* is faster because the heuristic guides search toward the goal. For our use case with clear start/end points, this reduces computational overhead.

### Why Altitude Stratification?
By assigning different altitudes to different traffic directions, we reduce the search space for conflicts. North/South drones physically cannot collide with East/West drones if they're at different altitudes.

### Why Speed Control Over Hovering?
Hovering consumes significant battery power. By adjusting speed, drones maintain forward momentum, which is more energy-efficient, especially with fixed-wing designs (future consideration).

## 🤝 Contributing

This is a PoC demonstration. For production use, consider:
- Integration with real drone APIs
- Regulatory compliance (FAA Part 107, etc.)
- Cybersecurity hardening
- High-availability architecture

## 📄 License

This project is a technical demonstration. Use at your own discretion.

## 🙏 Acknowledgments

Built with:
- [FastAPI](https://fastapi.tiangolo.com/) - Modern Python web framework
- [CesiumJS](https://cesium.com/platform/cesiumjs/) - 3D geospatial visualization
- [Pydantic](https://pydantic-docs.helpmanual.io/) - Data validation

---

**Status**: ✅ Fully Functional PoC  
**Last Updated**: 2026-02-12  
**Author**: Senior UAS Architect & Full-Stack Developer

# UTM System - Quick Reference Card

## 🚀 Quick Start (3 Steps)

### Step 1: Install Dependencies
```bash
pip install -r requirements.txt
```

### Step 2: Launch System
```bash
# Terminal 1: Start Backend
python main.py

# Terminal 2: Start Simulator
python drone_simulator.py

# Alternative: Use the launcher
bash start.sh
```

### Step 3: Open Browser
```
http://localhost:8000
```

---

## 📡 Key Endpoints

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/` | GET | Frontend UI |
| `/docs` | GET | API Documentation |
| `/api/delivery/request` | POST | Create delivery |
| `/api/drones` | GET | List all drones |
| `/api/missions` | GET | List missions |
| `/api/system/status` | GET | System stats |
| `/ws` | WebSocket | Real-time updates |

---

## 🎯 Example Delivery Request

```json
POST /api/delivery/request
{
  "pickup": {
    "latitude": 37.77,
    "longitude": -122.43,
    "altitude": 0
  },
  "delivery": {
    "latitude": 37.75,
    "longitude": -122.41,
    "altitude": 0
  }
}
```

---

## 🗺️ Coordinate Reference

**San Francisco Operational Area:**
- Lat: 37.60 to 37.80
- Lon: -122.45 to -122.35

**Pre-defined Zones:**
```
Airport (No-Fly):       37.6013 to 37.6213, -122.3790 to -122.3590
Military Base (No-Fly): 37.7650 to 37.7850, -122.4100 to -122.3900
School (Sensitive):     37.7650 to 37.7700, -122.4350 to -122.4300
Hospital (Sensitive):   37.7500 to 37.7550, -122.4050 to -122.4000
```

---

## ⚙️ Configuration Highlights

**File:** `config.py`

```python
# Performance
DRONE_MAX_SPEED = 15.0        # m/s
DRONE_CRUISE_SPEED = 10.0     # m/s

# Safety
HORIZONTAL_SEPARATION = 30.0  # meters
VERTICAL_SEPARATION = 15.0    # meters

# Altitude Lanes
DIRECTION_ALTITUDE_MAP = {
    'NORTH': [50, 90],        # m
    'SOUTH': [50, 90],        # m
    'EAST':  [30, 70, 110],   # m
    'WEST':  [30, 70, 110],   # m
}

# Simulation
SIMULATION_FLEET_SIZE = 10    # virtual drones
TELEMETRY_UPDATE_RATE = 2.0   # Hz
```

---

## 🧪 Test Scenarios

### Basic Delivery
```
Pickup:   (37.77, -122.43)
Delivery: (37.75, -122.41)
Expected: Direct path, no conflicts
```

### Multiple Conflicts
```
Create 5 deliveries simultaneously
Expected: Altitude stratification + speed adjustment
```

### No-Fly Zone Avoidance
```
Pickup:   (37.77, -122.43)
Delivery: (37.61, -122.37)
Expected: Path curves around airport
```

### Sensitive Area Cost
```
Pickup:   (37.78, -122.44)
Delivery: (37.74, -122.40)
Expected: Route avoids hospital (4x cost)
```

---

## 📊 System Architecture

```
┌─────────────────────┐
│   Frontend (Web)    │ ← User Interface
│   CesiumJS 3D Map   │
└──────────┬──────────┘
           │ WebSocket + REST
┌──────────▼──────────┐
│   Backend (API)     │ ← UTM Logic
│   • Pathfinding     │
│   • Geofencing      │
│   • Conflicts       │
└──────────┬──────────┘
           │ HTTP Telemetry
┌──────────▼──────────┐
│  Simulator (Fleet)  │ ← Virtual Drones
│   10 Mock Drones    │
└─────────────────────┘
```

---

## 🎨 Color Legend

| Color | Meaning |
|-------|---------|
| 🔴 Red | No-Fly Zones |
| 🟠 Orange | Sensitive Areas |
| 🔵 Cyan | Flight Paths |
| 🟢 Green | Active Drones |

---

## 🐛 Troubleshooting

**Problem:** Backend won't start  
**Solution:** Check if port 8000 is already in use
```bash
lsof -i :8000
kill -9 <PID>
```

**Problem:** Simulator can't connect  
**Solution:** Ensure backend is running first
```bash
curl http://localhost:8000/api/health
```

**Problem:** Drones not visible on map  
**Solution:** Check browser console, refresh page

**Problem:** No missions being created  
**Solution:** Check coordinates are within operational area

---

## 📚 File Structure

```
utm_system/
├── main.py                 ← Backend server
├── drone_simulator.py      ← Virtual fleet
├── pathfinding.py          ← A* algorithm
├── conflict_detection.py   ← Collision management
├── geofencing.py           ← Spatial queries
├── models.py               ← Data structures
├── config.py               ← All parameters
├── requirements.txt        ← Dependencies
├── start.sh                ← Launcher script
├── frontend/
│   └── index.html          ← 3D visualization
├── README.md               ← Full documentation
└── ARCHITECTURE.html       ← Visual diagram
```

---

## 🔑 Key Concepts

**4D Trajectory:** Path in 3D space (lat, lon, alt) over time  
**Conflict:** Two drones occupying same space at same time  
**Geofencing:** Spatial constraints (no-fly, sensitive areas)  
**Altitude Stratification:** Different altitudes for different directions  
**Speed Control:** Adjusting velocity to avoid conflicts  

---

## 📞 Support

For questions or issues:
1. Check `README.md` for detailed documentation
2. View `ARCHITECTURE.html` for visual overview
3. Access API docs at `http://localhost:8000/docs`

---

**Version:** 1.0.0  
**Status:** ✅ Production-Ready PoC  
**Last Updated:** 2026-02-12

# UTM System — Unmanned Traffic Management for Autonomous Drone Delivery

**A Proof of Concept for 4D Trajectory Management with GPS-Denied Navigation**

---

## Project Overview

A complete, production-ready Proof of Concept for an Unmanned Traffic Management (UTM) system designed for autonomous drone delivery at scale. The entire ecosystem is fully simulated — no physical hardware required.

A user sets a **Pickup** and **Delivery** location. The system automates everything else:

- 4D route planning (3D space + time) via real road networks (OSRM)
- Static conflict avoidance (geofencing — no-fly zones, sensitive areas)
- Dynamic conflict resolution (altitude stratification, takeoff delay)
- GPS-denied navigation via UWB anchor ranging + Weighted Least Squares correction
- Real-time 3D visualization in CesiumJS

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    FRONTEND (CesiumJS)                      │
│         3D Visualization + Control Interface                │
└────────────────┬────────────────────────────────────────────┘
                 │ WebSocket + REST API
┌────────────────▼────────────────────────────────────────────┐
│                   BACKEND (FastAPI)                         │
│  ┌────────────┐  ┌─────────────┐  ┌──────────────────────┐ │
│  │Pathfinding │  │ Geofencing  │  │ Conflict Resolution  │ │
│  │ (OSRM+A*)  │  │  Manager   │  │  (CPA + Stratum)     │ │
│  └────────────┘  └─────────────┘  └──────────────────────┘ │
│  ┌────────────┐  ┌─────────────┐                           │
│  │  Anchors   │  │ GPS-Denied  │                           │
│  │ (1km Grid) │  │ (WLS / UWB) │                           │
│  └────────────┘  └─────────────┘                           │
└─────────────────────────────────────────────────────────────┘
```

---

## File Structure

```
utm_system/
├── main.py                 ← FastAPI backend + telemetry loop
├── pathfinding.py          ← OSRM + A* 4D trajectory builder
├── conflict_detection.py   ← CPA conflict resolution (stratum + delay)
├── geofencing.py           ← No-fly zones, sensitive areas, water detection
├── anchors.py              ← Auto-generated 1 km UWB grid + GDOP selection
├── gps_denied.py           ← WLS position correction solver
├── models.py               ← Pydantic data models
├── config.py               ← All system parameters
├── drone_simulator.py      ← Legacy stub (no longer required)
├── requirements.txt        ← Python dependencies
├── start.sh                ← Launch script
├── .gitignore
├── QUICKSTART.md           ← Quick reference card
├── README.md               ← This file
├── ARCHITECTURE.html       ← Visual system diagram
└── frontend/
    ├── index.html          ← CesiumJS 3D visualization
    ├── app.js              ← Frontend logic + WebSocket client
    └── styles.css          ← UI styles
```

---

## Prerequisites

- Python 3.10+
- Modern web browser (Chrome, Firefox, Edge)
- Internet connection (CesiumJS CDN + OSRM public API)

---

## Installation & Setup

### Step 1 — Install dependencies

```bash
pip install -r requirements.txt
```

### Step 2 — Start the backend

```bash
python main.py
```

Expected output:
```
[Anchors] Grid registry built: 412 nodes at 1 km spacing over operational area
UTM System Starting...
INFO: Uvicorn running on http://0.0.0.0:8000
```

### Step 3 — Open the frontend

```
http://localhost:8000
```

> `drone_simulator.py` is no longer needed. Drones are spawned on-demand per delivery request.

---

## How to Use

### Standard Scenarios
Click any button in the **Standard Scenarios** panel. The drone flies a direct path with no conflicts.

### Reroute Demos
Click any button in the **Reroute Demos** panel. The system:
1. Shows the blocked straight-line path in red for 2.5 seconds
2. Replans via OSRM + A* around the no-fly zone
3. Flies the resolved route

### Custom Delivery
Enter pickup and delivery coordinates manually. The system rejects points over water (SF Bay or Pacific) with an inline warning before submission.

### Map Interactions
- **Click a drone** — opens the live inspection panel (altitude, speed, battery, UWB state)
- **All Routes toggle** — show/hide all flight paths simultaneously
- **Reroute Anim toggle** — enable/disable the blocked-path animation
- **UWB Ranging toggle** — show/hide all anchor-to-drone ranging lines globally

---

## Core Features

### 1. Pathfinding — 3-Level Fallback

| Level | Method | Condition |
|-------|--------|-----------|
| 1 | OSRM direct road route | Route is clear of no-fly zones |
| 2 | A* bypass → snap to road → OSRM chained | OSRM route blocked |
| 3 | Pure visibility-graph A* | OSRM unavailable |

After a 2D road path is found, `build_4d_trajectory()` inserts synthetic GROUND / TERMINAL / CLIMB / CRUISE / DESCENT waypoints with timing uncertainty (`sigma_t`) accumulated per waypoint.

### 2. Altitude Stratification

Direction-exclusive altitude lanes provide structural separation before any CPA logic runs:

```
WEST  → 30 m     EAST  → 50 m     (gap = 20 m)
SOUTH → 40 m     NORTH → 60 m     (gap = 20 m)
DIAGONAL → 80 m  (fallbacks: 100 m, 120 m)
```

### 3. Conflict Resolution — CPA Cascade

Conflict detection uses **continuous Closest-Point-of-Approach (CPA)** math on every pair of trajectory segments whose time windows overlap, eliminating discrete time-sampling blind spots.

Effective protection radius expands with timing uncertainty:
```
R_eff = R_BASE + speed_A × sigma_A + speed_B × sigma_B
```

Resolution cascade (applied in order):
1. **Altitude stratum change** — try nearest available stratum, greedy by GDOP cost
2. **Uniform takeoff delay** — binary-search minimum delay in `[sigma + margin, MAX_DELAY]`

Terminal areas (within 150 m of a pad, below 50 m) bypass CPA and use a FIFO pad queue instead.

### 4. GPS-Denied Navigation — UWB + WLS

**Anchor grid** — `anchors.py` auto-generates a uniform 1 km grid of UWB anchor nodes at startup over the full operational area. Water points (SF Bay, Pacific) are filtered out using the same shoreline model as `geofencing.py`. Node altitudes cycle through `[25, 45, 35, 55]` m in a checkerboard pattern for VDOP diversity.

**Node selection** — for each 4D waypoint, up to 6 anchor nodes are selected by greedy GDOP minimisation: seeded with the pair of maximum angular separation, then nodes added one-by-one by minimum combined GDOP.

**WLS correction** — at each waypoint checkpoint, UWB range measurements are simulated with distance-proportional Gaussian noise:
```
σᵢ = UWB_SIGMA_0 + UWB_DRIFT_K × dist_i
```

The correction vector is solved as:
```
Δp = −(HᵀWH)⁻¹ · Hᵀ · W · r
```

HDOP/VDOP gating:
- `HDOP > 3.0` → horizontal dead reckoning (Δx = Δy = 0)
- `VDOP > 3.0` → barometric altitude hold (Δz = 0)

**Position model** — the broadcast position is:
```
broadcast = planned_trajectory_position + WLS_correction_delta
```

The trajectory drives movement; towers correct accumulated drift. This mirrors real INS/UWB architecture.

---

## API Reference

Interactive docs at `http://localhost:8000/docs` once running.

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Frontend UI |
| `/api/health` | GET | System health check |
| `/api/system/status` | GET | Active missions, conflict counts |
| `/api/delivery/request` | POST | Create a delivery mission |
| `/api/missions` | GET | List all active missions |
| `/api/missions/{id}` | DELETE | Cancel a mission |
| `/api/airspace/status` | GET | Debug: all approved 4D flight plans |
| `/api/geofencing/zones` | GET | All geofence zone definitions |
| `/ws` | WebSocket | Real-time telemetry + mission events |

### Delivery Request

```json
POST /api/delivery/request
{
  "pickup":   { "latitude": 37.77, "longitude": -122.43, "altitude": 20 },
  "delivery": { "latitude": 37.75, "longitude": -122.41, "altitude": 20 }
}
```

### WebSocket Message Types

| Type | Direction | Description |
|------|-----------|-------------|
| `initial_state` | Server → Client | Geofencing + anchor node registry on connect |
| `mission_created` | Server → Client | New mission + trajectory + optional blocked path |
| `telemetry` | Server → Client | Position, battery, GPS-denied state at 2 Hz |
| `conflict_resolved` | Server → Client | Conflict resolution method + final stratum |
| `mission_complete` | Server → Client | Flight ended, entity cleanup signal |
| `mission_cancelled` | Server → Client | Operator cancel |

---

## Configuration (`config.py`)

### Drone Performance
| Parameter | Default | Description |
|-----------|---------|-------------|
| `DRONE_CRUISE_SPEED` | 35.0 m/s | Nominal cruise speed |
| `DRONE_MAX_SPEED` | 50.0 m/s | Hard speed limit |
| `DRONE_CLIMB_RATE` | 3.0 m/s | Vertical ascent rate |
| `DRONE_DESCENT_RATE` | 2.0 m/s | Vertical descent rate |
| `DRONE_MAX_ALTITUDE` | 130.0 m | Ceiling |

### Conflict Resolution
| Parameter | Default | Description |
|-----------|---------|-------------|
| `CPA_R_BASE` | 30.0 m | Static horizontal protection radius |
| `CPA_V_SEP` | 10.0 m | Minimum vertical separation |
| `MAX_TAKEOFF_DELAY` | 300.0 s | Maximum allowed delay before rejection |
| `TERMINAL_RADIUS` | 150.0 m | Pad FIFO zone radius |

### GPS-Denied / UWB
| Parameter | Default | Description |
|-----------|---------|-------------|
| `ANCHOR_MAX_RANGE` | 2500.0 m | UWB ranging radius |
| `ANCHOR_MIN_COUNT` | 4 | Minimum nodes for a valid WLS fix |
| `ANCHOR_MAX_COUNT` | 6 | Maximum nodes per waypoint (GDOP-optimised) |
| `HDOP_THRESHOLD` | 3.0 | Above this → horizontal dead reckoning |
| `VDOP_THRESHOLD` | 3.0 | Above this → barometric altitude hold |
| `UWB_SIGMA_0` | 0.5 m | Constant ranging noise floor |
| `UWB_DRIFT_K` | 0.005 m/m | Distance-proportional noise coefficient |

---

## Geofencing Zones

```
No-Fly (infinite cost):
  Airport Restricted Airspace  37.6013–37.6213, -122.3790–-122.3590
  Presidio Military Reserve    37.7900–37.8080, -122.5120–-122.4440

Sensitive Areas (elevated cost):
  Elementary School Zone  5× cost
  Hospital Complex        4× cost
  Residential High Density 2× cost
```

---

## Limitations & Future Work

| Area | Current State | Potential Enhancement |
|------|--------------|----------------------|
| Battery model | Linear discharge | Physics-based with wind, payload |
| Weather | None | Wind field integration |
| Mission type | Single pickup → delivery | Multi-leg, return-to-base |
| Conflict resolution | Stratum + delay | Full rerouting, holding patterns |
| UWB noise model | Gaussian isotropic | Multipath / NLOS modelling |
| Anchor grid | Static at startup | Dynamic deployment / failure simulation |

---

**Version:** 2.0.0  
**Status:** ✅ Fully Functional PoC  
**Last Updated:** 2026-03-04
# UTM System — Function Reference

> A complete dictionary of every function in the repository, organised by file.
> Each entry covers purpose, math/logic, parameters, return values, and a plain-English summary for complex functions.

---

## `config.py`

**Overview:** Centralised parameter store for the entire system. Contains no functions — only constants consumed by every other module. Changing a value here propagates instantly to pathfinding, conflict resolution, GPS-denied navigation, and the simulation loop.

---

## `models.py`

**Overview:** Defines all Pydantic data models and enumerations used as the shared data language between modules. No computation happens here — it is purely a schema file that enforces type safety across REST endpoints, WebSocket messages, and internal function calls.

---

## `geofencing.py`

**Overview:** Answers spatial questions about positions — is this point inside a forbidden zone, what does it cost to fly here, is a position over water? It is the single gatekeeper for all geographic constraint logic.

---

### `point_in_polygon(point, polygon)`

**Purpose:** Determine whether a (lat, lon) coordinate lies inside a polygon boundary.

**Math/Logic:** Implements the **ray-casting algorithm**. A horizontal ray is cast from the test point to the right (toward +∞ longitude). Every time the ray crosses an edge of the polygon, a boolean flag flips. If the flag is `True` after all edges are tested, the point is inside. Works for any simple (non-self-intersecting) polygon regardless of shape.

**Parameters:**
- `point` — `(latitude, longitude)` tuple to test
- `polygon` — list of `(latitude, longitude)` vertices in order

**Returns:** `bool` — `True` if the point is inside the polygon.

*Simple Terms: Imagines shooting a laser beam east from a point and counts how many times it crosses the zone boundary — odd crossings means you're inside.*

---

### `is_in_no_fly_zone(position)`

**Purpose:** Check whether a 3D position violates any no-fly zone, optionally respecting an altitude ceiling.

**Math/Logic:** Calls `point_in_polygon` for every zone in `config.NO_FLY_ZONES`. If the point falls inside a zone and the zone has no altitude ceiling (or the drone is below the ceiling), returns `True`.

**Parameters:**
- `position` — `Position` model (lat, lon, altitude)

**Returns:** `bool`

---

### `get_position_cost_multiplier(position)`

**Purpose:** Return a numeric cost multiplier for a position that the A* pathfinder uses to make certain areas expensive to cross.

**Math/Logic:** Returns `float('inf')` for no-fly zones (making them impassable in the graph). For sensitive areas (schools, hospitals), returns the highest applicable multiplier from `config.SENSITIVE_AREAS`. Returns `1.0` if the position is unrestricted.

**Parameters:**
- `position` — `Position` model

**Returns:** `float` — cost multiplier (1.0 = normal, >1.0 = discouraged, inf = prohibited)

---

### `is_trajectory_valid(waypoints)`

**Purpose:** Validate that an entire list of positions does not pass through any no-fly zone.

**Math/Logic:** Iterates over each waypoint and calls `is_in_no_fly_zone`. Fails fast on the first violation.

**Parameters:**
- `waypoints` — `List[Position]`

**Returns:** `Tuple[bool, str]` — `(True, "valid")` or `(False, error_message)`

---

### `get_safe_altitude_for_direction(heading, current_altitude)`

**Purpose:** Map a compass heading to the closest approved altitude lane (highway lane concept).

**Math/Logic:** Converts heading to a cardinal direction (N/S/E/W), looks up available altitudes from `config.DIRECTION_ALTITUDE_MAP`, and returns whichever altitude is numerically closest to `current_altitude`.

**Parameters:**
- `heading` — degrees 0–360
- `current_altitude` — current altitude in metres

**Returns:** `float` — recommended altitude in metres

---

### `is_within_operational_area(position)`

**Purpose:** Gate-check that a position falls inside the UTM system's bounding box before accepting it.

**Parameters:**
- `position` — `Position` model

**Returns:** `bool`

---

### `get_geofence_info()`

**Purpose:** Package all geofencing data for broadcast to the frontend WebSocket client.

**Returns:** `dict` with keys `no_fly_zones`, `sensitive_areas`, `operational_area`

---

### `_bay_threshold(lat)`

**Purpose:** For a given latitude, return the longitude of the SF Bay shoreline using a piecewise-linear approximation.

**Math/Logic:** Linearly interpolates between the pre-defined `_BAY_BOUNDARY` anchor points to get the longitude at which land transitions to bay water.

**Parameters:**
- `lat` — latitude

**Returns:** `float` — bay shoreline longitude at that latitude

---

### `is_over_water(lat, lon)`

**Purpose:** Determine whether a coordinate is over the Pacific Ocean or SF Bay, to prevent drones from being assigned to water locations.

**Math/Logic:** Two checks: if `lon <= _PACIFIC_LON` it is over the ocean; if `lon > _bay_threshold(lat)` it is over the bay.

**Parameters:**
- `lat`, `lon` — coordinate

**Returns:** `str | None` — description of water body, or `None` if on land

---

## `anchors.py`

**Overview:** Builds and manages the registry of fixed UWB infrastructure anchor nodes. At startup it auto-generates a uniform 1 km grid over the operational area, filters out water points, and exposes functions that select the geometrically optimal subset of nodes for any drone position.

---

### `_to_xyz(lat, lon, alt)`

**Purpose:** Convert geographic coordinates to a local flat-earth Cartesian frame centred at (37.75, −122.42).

**Math/Logic:** `x = (lat − ref_lat) × LAT_M`, `y = (lon − ref_lon) × LON_M × cos(ref_lat)`, `z = alt`. Accurate to < 0.1% over the ~40 km SF operational area.

**Parameters:**
- `lat`, `lon`, `alt` — geographic coordinates

**Returns:** `Tuple[float, float, float]` — (x, y, z) in metres

---

### `_is_over_water(lat, lon)`

**Purpose:** Water filter used during grid generation — mirrors `geofencing.is_over_water` without importing that module (avoids circular import).

**Returns:** `bool`

---

### `_build_grid_registry(grid_spacing_m)`

**Purpose:** Generate all anchor nodes at startup by walking a uniform lat/lon grid and discarding water points.

**Math/Logic:** Converts `grid_spacing_m` to degree steps using flat-earth factors. Cycles node altitudes through `[25, 45, 35, 55]` m using `(lat_idx + lon_idx) % 4` — the non-monotone order ensures every 2×2 block of adjacent nodes contains all four heights, maximising the vertical angular spread seen by any drone within 2.5 km (VDOP improvement).

**Parameters:**
- `grid_spacing_m` — node spacing in metres (default 1000)

**Returns:** `List[InfrastructureNode]`

---

### `get_anchor_registry()`

**Purpose:** Return the complete pre-built node list.

**Returns:** `List[InfrastructureNode]`

---

### `_unit_direction(drone_xyz, node)`

**Purpose:** Compute the unit vector pointing from the drone's position to a specific anchor node.

**Math/Logic:** `û = (node.xyz − drone.xyz) / ||node.xyz − drone.xyz||`. Returns `None` if the distance is negligibly small (drone is sitting on the node).

**Returns:** `Optional[Tuple[float, float, float]]`

---

### `_compute_gdop_split(drone_xyz, nodes)`

**Purpose:** Compute horizontal (HDOP) and vertical (VDOP) dilution of precision for a candidate set of anchor nodes.

**Math/Logic:** Builds the H matrix (n×3) where each row is `[ux, uy, uz]` — the unit direction to one node. Forms `HᵀH` (3×3). Inverts it analytically. `HDOP = √(inv[0][0] + inv[1][1])`, `VDOP = √(inv[2][2])`. Returns `(999, 999)` if fewer than `ANCHOR_MIN_COUNT` nodes or if the matrix is singular.

**Returns:** `Tuple[float, float]` — (hdop, vdop)

*Simple Terms: Measures how spread-out the towers are from the drone's perspective — towers clustered together on one side give a bad (high) score; towers surrounding the drone from all directions give a good (low) score.*

---

### `_invert_3x3(m)`

**Purpose:** Invert a 3×3 matrix analytically using cofactor expansion.

**Math/Logic:** Computes determinant `det = a(ek−fh) − b(dk−fg) + c(dh−eg)`. Returns `None` if `|det| < 1e-12` (singular). Constructs the inverse via the adjugate matrix divided by det.

**Parameters:**
- `m` — 3×3 matrix as `List[List[float]]`

**Returns:** `Optional[List[List[float]]]`

---

### `_angular_separation(drone_xyz, n1, n2)`

**Purpose:** Measure how far apart two nodes appear from the drone's viewpoint using the dot product of their unit direction vectors.

**Math/Logic:** `dot = û₁ · û₂`. Value of 1.0 means nodes are in the same direction (useless redundancy); value of −1.0 means they are on exactly opposite sides (maximum geometric diversity).

**Returns:** `float` — dot product in [−1, 1]

---

### `select_anchors(drone_xyz)`

**Purpose:** Choose the best 4–6 anchor nodes for a given drone position using greedy GDOP minimisation.

**Math/Logic:**
1. Filter to nodes within `ANCHOR_MAX_RANGE`.
2. If ≤ `ANCHOR_MAX_COUNT`, use all.
3. Otherwise: seed with the pair having the smallest dot product (largest angular gap). Greedily add whichever remaining node most reduces `√(HDOP² + VDOP²)` until `ANCHOR_MAX_COUNT` is reached.
4. Compute final HDOP/VDOP.

**Returns:** `Tuple[List[InfrastructureNode], float, float]` — (selected_nodes, hdop, vdop)

*Simple Terms: Picks the handful of towers that give the drone the best geometric "surround" — like choosing witnesses positioned around a crime scene rather than all standing on the same side.*

---

### `compute_anchor_bindings(drone_xyz)`

**Purpose:** Public entry point called by `pathfinding.build_4d_trajectory` at planning time. Selects optimal nodes and pre-computes the expected 3D Euclidean distance to each one.

**Math/Logic:** Calls `select_anchors`, then for each selected node: `dist = √((node.x−dx)² + (node.y−dy)² + (node.z−dz)²)`. Stores result as `AnchorBinding(node_id, expected_dist_m)`.

**Returns:** `Tuple[List[AnchorBinding], float, float]` — (bindings, hdop, vdop)

---

## `gps_denied.py`

**Overview:** Implements the GPS-denied navigation correction engine. At each waypoint checkpoint it synthesises noisy UWB range measurements and runs a Weighted Least Squares (WLS) solver to compute a position correction vector that counteracts accumulated inertial drift.

---

### `simulate_ranging(bindings, true_xyz)`

**Purpose:** Synthesise realistic noisy UWB distance measurements for a drone that has drifted from its planned position.

**Math/Logic:** For each anchor binding: compute the true Euclidean distance from the *actual* drifted position. Add Gaussian noise scaled by distance: `σᵢ = UWB_SIGMA_0 + UWB_DRIFT_K × true_dist`. Measured distance = `true_dist + N(0, σᵢ)`, clamped positive.

**Parameters:**
- `bindings` — `List[AnchorBinding]` from the current waypoint's plan
- `true_xyz` — actual (drifted) drone position in Cartesian metres

**Returns:** `List[Tuple[str, float, float]]` — list of `(node_id, expected_dist, measured_dist)`

*Simple Terms: Pretends to be a UWB radio — it knows how far away each tower really is but adds a small realistic measurement error to simulate a real sensor.*

---

### `_build_H_and_residuals(bindings, measurements, planned_xyz)`

**Purpose:** Construct the H matrix, residual vector, and weight vector needed for the WLS solve.

**Math/Logic:**
- Unit direction `ûᵢ` from planned drone position to node i.
- Residual `rᵢ = measured_i − expected_i` (the raw range error).
- Weight `wᵢ = 1/σᵢ²` where `σᵢ = UWB_SIGMA_0 + UWB_DRIFT_K × expected_i`. Closer nodes get higher weight because their measurements are more precise.

**Returns:** `Tuple[List[...], List[float], List[float]]` — (H_rows, residuals, weights)

---

### `_gdop_from_H(H_rows)`

**Purpose:** Compute HDOP and VDOP from already-built H matrix rows (avoids rebuilding from InfrastructureNode objects).

**Math/Logic:** Identical to `anchors._compute_gdop_split` but operates on pre-built rows. Used to gate the WLS solve — if geometry is poor, the correction is zeroed out rather than amplified.

**Returns:** `Tuple[float, float]` — (hdop, vdop)

---

### `_solve_wls(H_rows, residuals, weights)`

**Purpose:** Solve the core WLS position correction equation.

**Math/Logic:** The linearised ranging equation is `H · Δp ≈ −r`, so the displacement is:

```
Δp = −(HᵀWH)⁻¹ · Hᵀ · W · r
```

where W is the diagonal weight matrix. Steps:
1. Form `HᵀWH` (3×3): `HᵀWH[i][j] = Σₖ wₖ · H[k][i] · H[k][j]`
2. Invert via `_invert_3x3`
3. Form `HᵀWr` (3×1): `HᵀWr[i] = Σₖ wₖ · H[k][i] · rₖ`
4. `Δp = −(HᵀWH)⁻¹ · HᵀWr`

**Returns:** `Optional[Tuple[float, float, float]]` — (Δx, Δy, Δz) in metres, or `None` if singular

*Simple Terms: Given that each tower is reporting a slightly different distance than expected, this solves "how far and in which direction must the drone have actually drifted?" using a trust-weighted average that favours nearby towers.*

---

### `_compute_residual_rms(H_rows, residuals, delta_xyz)`

**Purpose:** Measure the quality of a WLS fix by computing the root-mean-square of the ranging errors after the correction is applied.

**Math/Logic:** Post-correction residual for node i: `eᵢ = rᵢ − ûᵢ · Δp`. `RMS = √(Σ eᵢ² / n)`. A low RMS means the correction explains the measurements well; a high RMS suggests multipath or a poor geometry fix.

**Returns:** `float` — RMS in metres

---

### `compute_wls_correction(bindings, measurements, planned_xyz)`

**Purpose:** Top-level public function that assembles the WLS pipeline and applies HDOP/VDOP gating to produce a safe correction vector.

**Math/Logic:**
1. Build H, residuals, weights via `_build_H_and_residuals`.
2. Compute HDOP/VDOP — gate the solve.
3. If `HDOP > threshold`: zero out Δx, Δy (horizontal dead reckoning).
4. If `VDOP > threshold`: zero out Δz (barometric altitude hold).
5. Velocity correction: `v_correction = Δp / τ` where τ = `WLS_CORRECTION_TAU`.
6. Compute residual RMS.

**Parameters:**
- `bindings` — `List[AnchorBinding]` (planned distances from waypoint)
- `measurements` — output of `simulate_ranging`
- `planned_xyz` — drone's planned position in Cartesian metres

**Returns:** `WLSResult` — contains `delta_xyz`, `velocity_correction`, `hdop_ok`, `vdop_ok`, `n_nodes`, `residual_rms`

*Simple Terms: The central GPS-denied brain — it takes "here's what the towers measured vs. what we expected" and converts that into "move this many metres in this direction to correct your drift, but only if the tower geometry is good enough to trust."*

---

## `pathfinding.py`

**Overview:** Converts a pickup/delivery coordinate pair into a full 4D trajectory using a three-level routing pipeline (OSRM roads → Yen's K=5 bypass → pure visibility graph). Handles the climb/cruise/descent phase structure, turn-speed modelling, and anchor binding pre-computation.

---

### `haversine_distance(lat1, lon1, lat2, lon2)`

**Purpose:** Compute the great-circle surface distance between two geographic points.

**Math/Logic:** `a = sin²(Δφ/2) + cos φ₁ · cos φ₂ · sin²(Δλ/2)`, `d = 2R · atan2(√a, √(1−a))` where R = 6,371,000 m. Accurate for distances up to thousands of km.

**Returns:** `float` — metres

---

### `distance_3d(lat1, lon1, alt1, lat2, lon2, alt2)`

**Purpose:** Euclidean 3D distance combining horizontal haversine distance and vertical altitude difference.

**Math/Logic:** `d3 = √(haversine² + Δalt²)`

**Returns:** `float` — metres

---

### `calculate_heading(lat1, lon1, lat2, lon2)`

**Purpose:** Compute the forward bearing (compass heading) from point 1 to point 2.

**Math/Logic:** Uses atan2 on the cross-track formula: `x = sin(Δλ)cos(φ₂)`, `y = cos(φ₁)sin(φ₂) − sin(φ₁)cos(φ₂)cos(Δλ)`, `heading = (degrees(atan2(x, y)) + 360) % 360`.

**Returns:** `float` — degrees, 0 = North, 90 = East

---

### `interpolate_waypoint(start, end, fraction)`

**Purpose:** Linear interpolation between two `Position` objects at a given fractional distance.

**Returns:** `Position`

---

### `_ccw(A, B, C)` / `_segments_intersect(A, B, C, D)`

**Purpose:** Computational geometry primitives for line-segment intersection testing.

**Math/Logic:** `_ccw` checks the sign of the cross product of vectors AB and AC. `_segments_intersect` uses the standard "two segments intersect iff their endpoints straddle each other" test derived from `_ccw`.

---

### `_segment_crosses_polygon(p1, p2, polygon)`

**Purpose:** Determine if a flight path segment enters or crosses a no-fly zone polygon.

**Math/Logic:** Returns `True` if either endpoint is inside the polygon (via `point_in_polygon`), or if any polygon edge intersects the segment (via `_segments_intersect`).

---

### `_path_hits_no_fly_zone(waypoints)` / `_segment_is_clear(p1, p2)`

**Purpose:** Batch validation of a full path or single segment against all no-fly zones.

**Returns:** `bool`

---

### `fetch_osrm_route(start, goal)`

**Purpose:** Request a real-world road-following route from the public OSRM routing engine.

**Math/Logic:** HTTP GET to `router.project-osrm.org`. Parses GeoJSON coordinates from the response. OSRM uses Contraction Hierarchies on OpenStreetMap data to produce car-navigable routes.

**Returns:** `Optional[List[LatLon]]`

---

### `snap_to_road(lat, lon)`

**Purpose:** Find the nearest road-accessible point to an arbitrary coordinate using OSRM's nearest endpoint.

**Returns:** `Optional[LatLon]`

---

### `fetch_osrm_route_via_positions(positions)`

**Purpose:** Request a chained OSRM route through multiple waypoints — used when bypass corners need to be road-snapped.

**Returns:** `Optional[List[LatLon]]`

---

### `_turn_speed(prev_heading, next_heading, cruise, min_speed)`

**Purpose:** Reduce cruise speed proportionally to the sharpness of a direction change.

**Math/Logic:** Turn angle `δ = |heading₂ − heading₁|` normalised to [0°, 180°]. Speed = `cruise × 0.5 × (1 + cos(δ))`, clamped to `[min_speed, cruise]`. The cosine mapping gives full speed at 0°, half speed at 90°, and `min_speed` at 180°.

**Parameters:**
- `prev_heading`, `next_heading` — incoming and outgoing headings in degrees
- `cruise` — nominal cruise speed m/s
- `min_speed` — floor speed m/s

**Returns:** `float` — turn-adjusted speed in m/s

*Simple Terms: The drone slows down for corners just like a car would — a gentle bend barely reduces speed but a sharp U-turn brings it nearly to minimum.*

---

### `get_path_travel_time(latlon_path)`

**Purpose:** Estimate total flight time for a 2D path accounting for both segment distances and corner deceleration penalties.

**Math/Logic:** For each segment: `base_time = dist / turn_speed`. Additionally, `accel_penalty = 2 × (cruise − turn_speed) / DRONE_ACCEL_LIMIT` accounts for the time to decelerate into the corner and re-accelerate out. `total = Σ(base_time + penalty)`.

**Returns:** `float` — estimated seconds

*Simple Terms: The stopwatch for a path — longer paths take more time, but a short path with many sharp turns can take longer than a longer but straighter one.*

---

### `_buffered_vertices(polygon)`

**Purpose:** Expand a no-fly zone polygon outward by `BUFFER_DEGREES` so visibility-graph bypass nodes give physical clearance from the zone boundary.

**Math/Logic:** For each vertex, computes the vector from the polygon centroid and scales it by `(d + BUFFER_DEGREES) / d`.

**Returns:** `List[LatLon]`

---

### `_edge_cost(p1, p2)`

**Purpose:** Compute the A* graph edge cost for a visibility-graph segment, incorporating geofencing penalties.

**Math/Logic:** `cost = haversine(p1, p2) × get_position_cost_multiplier(midpoint)`. No-fly zone midpoints return `inf`, making those edges impassable.

**Returns:** `float`

---

### `_astar_on_graph(nodes, blocked_edges, blocked_nodes, start_idx, goal_idx)`

**Purpose:** Run A* on a pre-built node set with dynamically blocked edges and nodes — the inner engine for Yen's algorithm.

**Math/Logic:** Standard A* with `g(n) = cumulative edge cost`, `h(n) = haversine to goal`. Skips any edge in `blocked_edges` (frozenset of index pairs) or either endpoint in `blocked_nodes`. Visibility check via `_segment_is_clear` prunes edges that cross no-fly zones.

**Parameters:**
- `nodes` — full list of candidate LatLon points
- `blocked_edges` — set of `frozenset({i, j})` pairs to ignore
- `blocked_nodes` — set of node indices to skip

**Returns:** `Optional[List[int]]` — ordered node indices from start to goal

---

### `visibility_graph_astar(start, goal)`

**Purpose:** Find the time-optimal path using Yen's K=5 algorithm — generates up to 5 distinct loopless paths and selects the fastest.

**Math/Logic (Yen's Algorithm):**
1. Build node set: start, goal, buffered polygon vertices.
2. `A[0]` = shortest path by edge cost.
3. For k = 1…K−1:
   - For each spur node index i in `A[k-1]`:
     - Root = `A[k-1][0…i]`
     - Block edges departing spur_node that were used by any confirmed path with the same root prefix
     - Block all nodes in root except spur_node
     - Run `_astar_on_graph` from spur_node to goal
     - If found: `candidate = root + spur_path`; add to min-heap B if not duplicate
   - `A[k]` = lowest-cost candidate popped from B
4. Score all paths in A with `get_path_travel_time()`.
5. Return the lowest-time path.

**Returns:** `Optional[List[LatLon]]`

*Simple Terms: Instead of finding just one route, finds up to 5 different routes around obstacles, times each one including slowdowns for corners, and picks the one that gets the drone there fastest.*

---

### `_thin_path(points, min_dist_m)`

**Purpose:** Remove redundant intermediate waypoints that are closer than `min_dist_m` to their predecessor, reducing trajectory size without changing the route shape.

**Returns:** `List[LatLon]`

---

### `bearing_to_direction(heading)` / `direction_to_stratum(direction)`

**Purpose:** Map a compass bearing to a cardinal direction string, then to the assigned cruise altitude stratum from `config.DIRECTION_STRATA`.

**Returns:** `str` direction / `int` stratum altitude metres

---

### `_is_in_terminal_area(lat, lon, alt, pad_lat, pad_lon)`

**Purpose:** Check whether a position is inside the cylindrical terminal area around a pad (radius = `TERMINAL_RADIUS`, height = `TERMINAL_HEIGHT`). Terminal waypoints bypass CPA checks and are governed by the pad queue instead.

**Returns:** `bool`

---

### `_compute_sigma_t(cumulative_arc_m)`

**Purpose:** Calculate timing uncertainty for a waypoint based on how far from launch it is.

**Math/Logic:** `σ_t = CPA_SIGMA_0 + CPA_DRIFT_K × arc_metres`. Uncertainty grows linearly with distance because timing errors accumulate over longer flights. Used to expand the effective CPA protection radius.

**Returns:** `float` — seconds of timing uncertainty

---

### `build_4d_trajectory(latlon_path, pickup_pos, delivery_pos, stratum, direction, start_time)`

**Purpose:** Convert a flat 2D road path into a complete 4D trajectory with phase-tagged waypoints, accurate ETAs, sigma_t values, and pre-computed anchor bindings.

**Math/Logic:**
1. Compute horizontal distances covered during climb and descent at cruise speed.
2. Walk path forward to find the climb endpoint; walk backward for descent start.
3. Insert GROUND → CLIMB → CRUISE → DESCENT → GROUND segments.
4. For each segment: compute heading, apply `_turn_speed` for CRUISE waypoints, compute `seg_time` using the turn-adjusted speed, accumulate `current_time` and `cumulative_arc`.
5. At each waypoint call `anchor_registry.compute_anchor_bindings` for the GPS-denied navigation pre-computation.

**Returns:** `Trajectory4D`

---

### `get_direct_path(start, goal)`

**Purpose:** Return the straight-line path between two points for the reroute animation — shows the blocked direct route before the real path is revealed.

**Returns:** `List[LatLon]` — just two points

---

### `plan_trajectory(start, goal, start_time, stratum, direction)`

**Purpose:** Top-level public function orchestrating the three-level routing pipeline.

**Logic:**
- Level 1: OSRM direct → validate against no-fly zones
- Level 2: Yen's bypass corners → snap to roads → OSRM chained
- Level 3: Pure Yen's K=5 (no road guarantee)

Assigns stratum and direction from overall heading if not provided, then calls `build_4d_trajectory`.

**Returns:** `Optional[Trajectory4D]`

---

## `conflict_detection.py`

**Overview:** Implements preventive 4D conflict resolution applied entirely at planning time. Uses continuous Closest-Point-of-Approach (CPA) mathematics on trajectory segments to detect collisions, then resolves them through an altitude stratum cascade followed by binary-searched takeoff delays.

---

### `_to_metres(lat, lon, alt, ref_lat)`

**Purpose:** Convert geographic coordinates to the local Cartesian metric frame used throughout conflict detection.

**Math/Logic:** Same flat-earth projection as `anchors._to_xyz` — centred at (37.75, −122.42).

**Returns:** `Vec3` — (x, y, z) tuple in metres

---

### `_dist3(a, b)` / `_dist2(a, b)`

**Purpose:** Euclidean 3D and horizontal-only 2D distances between two Cartesian vectors.

---

### `PadQueue.earliest_available(lat, lon, not_before)`

**Purpose:** Query the FIFO pad queue for the earliest time a pad is free, accounting for `PAD_CLEARANCE_TIME` gaps between consecutive users.

**Math/Logic:** Iterates existing reservations sorted by start time. If `not_before` overlaps any reservation window plus its clearance buffer, advances `not_before` past that window. Returns the final candidate time.

**Returns:** `float` — Unix timestamp

---

### `PadQueue.reserve(drone_id, lat, lon, window_start, window_end)`

**Purpose:** Register a drone's time slot on a pad, keyed by a rounded (lat, lon) grid cell.

---

### `PadQueue.release(drone_id)`

**Purpose:** Remove all pad reservations for a drone after its mission completes or is cancelled.

---

### `_segment_cpa(a0, a1, ta0, ta1, b0, b1, tb0, tb1)`

**Purpose:** Find the minimum 3D separation between two linearly-moving drones over their shared time window — the mathematical heart of the conflict detection system.

**Math/Logic:** Both drones move linearly within their segment. The squared distance between them is a quadratic in time `t`: `d²(t) = |r₀ + v_rel · (t − t_lo)|²`. Setting the derivative to zero gives the analytic CPA time `t* = t_lo − (r₀ · v_rel) / |v_rel|²`, clamped to the shared window `[t_lo, t_hi]`. Evaluates position of both drones at `t*` to get minimum horizontal and vertical distances.

**Parameters:**
- `a0`, `a1` — start/end Cartesian positions of drone A's segment
- `ta0`, `ta1` — start/end ETAs of drone A's segment
- `b0`, `b1`, `tb0`, `tb1` — same for drone B

**Returns:** `Tuple[float, float, float, float]` — (min_3d_dist, min_horiz_dist, min_vert_dist, cpa_time)

*Simple Terms: Given two drones flying in straight lines, finds the single moment in time when they are closest to each other, then measures exactly how close that is — without needing to check every second.*

---

### `_effective_radius(wp_a, wp_b)`

**Purpose:** Compute the dynamic horizontal protection radius between two drones, expanding it when timing uncertainty is high.

**Math/Logic:** `R_eff = R_BASE + speed_A × σ_A + speed_B × σ_B`. A drone with higher speed and higher timing uncertainty could be further from its expected position, so the safety bubble must be larger.

**Returns:** `float` — metres

---

### `apply_delay(traj, extra_delay)`

**Purpose:** Shift all ETAs in a trajectory forward by `extra_delay` seconds without changing any positions or anchor bindings.

**Math/Logic:** Adds `extra_delay` to every `wp.eta` and to `traj.takeoff_delay`. Anchor bindings are position-based and are not affected by time shifts — this is correct because the bindings encode expected distances, not times.

**Returns:** `Trajectory4D`

---

### `reassign_stratum(traj, new_stratum, pickup_pos, delivery_pos)`

**Purpose:** Rebuild a trajectory at a different cruise altitude, rescaling all climb and descent waypoints proportionally and recomputing anchor bindings at the new altitudes.

**Math/Logic:** CRUISE waypoints set to `new_stratum`. CLIMB/DESCENT/TERMINAL waypoints scaled: `new_alt = (old_alt / old_stratum) × new_stratum`. Calls `anchor_registry.compute_anchor_bindings` fresh for every waypoint — critical because the old bindings had `expected_dist_m` computed at the previous altitude, which would bias the WLS residuals if reused.

**Returns:** `Trajectory4D`

---

### `check_trajectory_vs_all(candidate, existing)`

**Purpose:** Check a new trajectory against every already-approved trajectory for 4D conflicts.

**Math/Logic:** For every pair of overlapping time-window segments (one from candidate, one from each existing trajectory), calls `_segment_cpa`. A conflict is recorded if `d_h < R_eff AND d_v < CPA_V_SEP`. Terminal/ground segment pairs are skipped (handled by pad queue instead).

**Parameters:**
- `candidate` — `Trajectory4D` being evaluated
- `existing` — `Dict[str, Trajectory4D]` of all approved flights

**Returns:** `List[ConflictInfo]` — empty = clear airspace

---

### `ConflictResolver.resolve(candidate, existing, pickup_pos, delivery_pos)`

**Purpose:** The top-level planning-time conflict resolver. Runs the full cascade until the airspace is clear or the resolution budget is exhausted.

**Math/Logic (cascade loop):**
1. `check_trajectory_vs_all` → if empty, done.
2. Sort conflicts by earliest CPA time. Take the worst.
3. **Resolution 1:** Try each alternate stratum (nearest first) via `reassign_stratum`. If any clears all conflicts, adopt it.
4. **Resolution 2:** If all strata blocked, binary-search the minimum takeoff delay in `[σ_combined + TEMPORAL_SAFETY_MARGIN, MAX_TAKEOFF_DELAY]` via `apply_delay`.
5. Re-validate on the next pass. Repeat up to `MAX_RESOLUTION_PASSES` times.
6. If still unresolved, raise `AirspaceSaturatedError`.

**Returns:** `Tuple[Trajectory4D, Optional[ConflictAlert]]`

*Simple Terms: The air traffic controller — it looks at where every drone plans to be at every moment, and if two will be too close, it either moves one to a different altitude lane or makes it wait a few minutes before taking off.*

---

### `ConflictResolver._binary_search_delay(traj, existing, lo, hi)`

**Purpose:** Find the minimum takeoff delay that resolves all conflicts using binary search.

**Math/Logic:** Tests `mid = (lo + hi) / 2`; if `check_trajectory_vs_all(apply_delay(traj, mid))` still has conflicts, set `lo = mid`; else `hi = mid`. Repeats 8 iterations. Returns `hi` — the smallest clean delay found.

**Returns:** `Optional[float]` — delay in seconds, or `None` if even `hi` doesn't help

---

## `main.py`

**Overview:** The FastAPI application server and mission lifecycle manager. Receives delivery requests, orchestrates the planning pipeline across all other modules, manages active mission state, runs the 2 Hz telemetry simulation loop, and broadcasts all events to connected WebSocket clients.

---

### `_interpolate_position(traj, t)`

**Purpose:** Linearly interpolate a drone's geographic position along its legacy trajectory at wall-clock time `t`.

**Math/Logic:** Walks waypoints to find the bracketing segment `[w0, w1]` where `w0.eta ≤ t ≤ w1.eta`. `r = (t − w0.eta) / (w1.eta − w0.eta)`. Position = `w0 + r × (w1 − w0)` for each coordinate.

**Returns:** `Optional[Position]`

---

### `_estimate_battery(traj, t)`

**Purpose:** Estimate remaining battery percentage at time `t` using linear discharge.

**Math/Logic:** `fraction = elapsed / total_flight_time`. `battery = 100 − fraction × estimated_battery_usage`.

**Returns:** `float` — percentage 0–100

---

### `_telemetry_loop()`

**Purpose:** Async background task that broadcasts position and GPS-denied navigation state for every active drone at 2 Hz.

**Logic:**
1. For each active mission, call `_interpolate_position` to get planned position.
2. Detect if a new 4D waypoint checkpoint has been reached (by comparing `current_wp_idx > prev_wp_idx`).
3. If new checkpoint: simulate IMU drift, call `gps_denied.simulate_ranging`, then `gps_denied.compute_wls_correction`. Persist result in `_wls_state[drone_id]`.
4. Apply correction within the `WLS_CORRECTION_TAU` window using linear fade-out.
5. Broadcast corrected position + GPS-denied telemetry via WebSocket.

---

### `create_delivery(request)` — `POST /api/delivery/request`

**Purpose:** The main planning pipeline endpoint. Orchestrates 9 sequential steps from input validation to mission broadcast.

**Logic:**
1. Validate bounds, no-fly zones, water
2. Compute straight-line for reroute animation
3. `plan_trajectory` (OSRM + Yen's + 4D builder) in thread pool
4. Pad queue reservation (if `PAD_QUEUE_ENABLED`)
5. `ConflictResolver.resolve` in thread pool
6. Re-reserve pads with final ETAs
7. Build `Mission` object, store in state
8. Broadcast `mission_created` via WebSocket
9. Schedule `_cleanup_mission` task

**Returns:** Mission summary JSON

---

### `_cleanup_mission(mission_id, drone_id, delay_s)`

**Purpose:** Async task that fires after a flight ends to remove the drone from all state dictionaries, release pad reservations, and broadcast `mission_complete` to the frontend.

---

### `websocket_endpoint(ws)`

**Purpose:** Handle WebSocket connections. On connect: push `initial_state` (geofencing + anchor registry). Replay all active missions for late-joining clients. Keep connection alive with 20 s pings.

---

## `drone_simulator.py`

**Overview:** Legacy stub — prints a message directing users to run `main.py` instead. Drones are now spawned on-demand per delivery request inside the telemetry loop.

---

*End of Function Reference*
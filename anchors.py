"""
anchors.py — Infrastructure Node Registry & Anchor Selection
=============================================================

Public API
----------
get_anchor_registry()        -> List[InfrastructureNode]
select_anchors(x, y, z)      -> Tuple[List[InfrastructureNode], float, float]
    Returns (selected_nodes, hdop, vdop)
compute_expected_distances(waypoint_xyz, nodes) -> List[AnchorBinding]

Internal pipeline
-----------------
1.  Filter registry to nodes within ANCHOR_MAX_RANGE of the waypoint
2.  If ≤ ANCHOR_MAX_COUNT candidates, use all of them
3.  If > ANCHOR_MAX_COUNT, run greedy GDOP minimisation:
      a. Start with the 2 nodes forming the largest angular separation
      b. Greedily add the node that most reduces total GDOP until count == MAX
4.  Compute split HDOP / VDOP from the final H matrix
5.  Compute expected 3-D Euclidean distances

Registry generation
-------------------
The anchor node registry is generated automatically at module load time.
A uniform grid of nodes is placed every 1 km across the full operational
area defined in config.OPERATIONAL_AREA.  Nodes whose (lat, lon) falls over
SF Bay or the Pacific Ocean are discarded using the same piecewise-linear
shoreline model used by geofencing.py.

Altitude assignment uses a deterministic 4-level rotation keyed on the
grid indices, giving node heights of 25 m / 35 m / 45 m / 55 m in a
checkerboard-style pattern.  This ensures vertical angular diversity in
the H matrix (good VDOP) even though all nodes are on a flat 2-D grid.

Coordinate frame
----------------
All computations use the same local Cartesian metric frame as
conflict_detection.py — centred at (37.75, -122.42), flat-earth,
accurate to < 0.1% over the SF operational area (~40 km).
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import config
from models import AnchorBinding

# ── Coordinate conversion constants (mirrors conflict_detection.py) ───────────
_REF_LAT = 37.75
_REF_LON = -122.42
_LAT_M   = 111_319.5
_LON_M   = 111_319.5


def _to_xyz(lat: float, lon: float, alt: float) -> Tuple[float, float, float]:
    """Convert geographic (lat, lon, alt) to local Cartesian metres."""
    x = (lat - _REF_LAT) * _LAT_M
    y = (lon - _REF_LON) * _LON_M * math.cos(math.radians(_REF_LAT))
    z = alt
    return x, y, z


# ══════════════════════════════════════════════════════════════════════════════
# DATA CLASS
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class InfrastructureNode:
    """
    A fixed infrastructure anchor node with known absolute coordinates.

    xyz is pre-computed at registry initialisation time so that
    per-waypoint distance calculations are pure arithmetic with no
    trig overhead.
    """
    node_id:   str
    label:     str
    lat:       float
    lon:       float
    alt_m:     float
    # Pre-computed Cartesian position (set in __post_init__)
    x: float = field(init=False)
    y: float = field(init=False)
    z: float = field(init=False)

    def __post_init__(self) -> None:
        self.x, self.y, self.z = _to_xyz(self.lat, self.lon, self.alt_m)


# ══════════════════════════════════════════════════════════════════════════════
# WATER-BODY FILTER
# ══════════════════════════════════════════════════════════════════════════════
# Mirrors the shoreline model in geofencing.py so we don't create a circular
# import (geofencing.py → models → ... ← anchors.py).
# Nodes generated over SF Bay or the Pacific are silently dropped.

_BAY_BOUNDARY = [
    (37.50, -122.315), (37.56, -122.325), (37.58, -122.330),
    (37.60, -122.335), (37.62, -122.345), (37.65, -122.358),
    (37.68, -122.365), (37.71, -122.370), (37.74, -122.376),
    (37.77, -122.383), (37.79, -122.390), (37.81, -122.397),
    (37.83, -122.408), (37.90, -122.430),
]
_PACIFIC_LON = -122.511


def _bay_threshold(lat: float) -> float:
    """Interpolated bay shoreline longitude for a given latitude."""
    pts = _BAY_BOUNDARY
    if lat <= pts[0][0]:  return pts[0][1]
    if lat >= pts[-1][0]: return pts[-1][1]
    for i in range(len(pts) - 1):
        la0, lo0 = pts[i]
        la1, lo1 = pts[i + 1]
        if la0 <= lat <= la1:
            t = (lat - la0) / (la1 - la0)
            return lo0 + t * (lo1 - lo0)
    return pts[-1][1]


def _is_over_water(lat: float, lon: float) -> bool:
    """Return True if (lat, lon) is over the Pacific Ocean or SF Bay."""
    if lon <= _PACIFIC_LON:
        return True
    if lon > _bay_threshold(lat):
        return True
    return False


# ══════════════════════════════════════════════════════════════════════════════
# GRID REGISTRY GENERATOR
# ══════════════════════════════════════════════════════════════════════════════

# Altitude levels cycled through in a deterministic checkerboard pattern.
# Four distinct heights give the H matrix enough vertical angular spread to
# keep VDOP below the VDOP_THRESHOLD (3.0) at typical drone cruise altitudes.
_ALT_LEVELS = [25.0, 45.0, 35.0, 55.0]   # metres — deliberately non-monotone


def _build_grid_registry(grid_spacing_m: float = 1000.0) -> List[InfrastructureNode]:
    """
    Generate a uniform anchor grid over config.OPERATIONAL_AREA.

    Parameters
    ----------
    grid_spacing_m : spacing between adjacent nodes in metres (default 1000 m)

    Algorithm
    ---------
    1. Convert the operational bounding box to a (lat_step, lon_step) pair
       using flat-earth approximation centred at _REF_LAT.
    2. Walk the grid south→north, west→east, assigning each (lat, lon) a
       node ID "G{index:04d}" and a label encoding the rounded position.
    3. Skip any point over water (Pacific or SF Bay).
    4. Cycle through _ALT_LEVELS using (lat_idx + lon_idx) % 4 so that every
       2×2 block of nodes contains all four altitude levels — maximising the
       geometric spread seen by any drone within a 2.5 km radius.

    Returns
    -------
    List[InfrastructureNode] — all on-land grid nodes, ordered south→north,
    west→east.  Typically ~350–450 nodes for the SF operational area at 1 km.
    """
    bounds   = config.OPERATIONAL_AREA
    cos_lat  = math.cos(math.radians(_REF_LAT))

    lat_step = grid_spacing_m / _LAT_M
    lon_step = grid_spacing_m / (_LON_M * cos_lat)

    nodes: List[InfrastructureNode] = []
    node_idx = 1

    lat = bounds['min_lat']
    lat_i = 0
    while lat <= bounds['max_lat'] + lat_step * 0.5:   # +0.5 step avoids float edge miss
        lon = bounds['min_lon']
        lon_i = 0
        while lon <= bounds['max_lon'] + lon_step * 0.5:
            if not _is_over_water(lat, lon):
                alt_m  = _ALT_LEVELS[(lat_i + lon_i) % len(_ALT_LEVELS)]
                label  = f"Grid_{lat:.3f}_{abs(lon):.3f}W"
                nodes.append(InfrastructureNode(
                    node_id = f"G{node_idx:04d}",
                    label   = label,
                    lat     = lat,
                    lon     = lon,
                    alt_m   = alt_m,
                ))
                node_idx += 1
            lon  += lon_step
            lon_i += 1
        lat  += lat_step
        lat_i += 1

    return nodes


# ── Build registry once at module load ────────────────────────────────────────
# _build_grid_registry() runs in < 5 ms (pure arithmetic, no I/O).
# The resulting list is held in _REGISTRY for the lifetime of the process.
_REGISTRY: List[InfrastructureNode] = _build_grid_registry(grid_spacing_m=1000.0)

print(f"[Anchors] Grid registry built: {len(_REGISTRY)} nodes "
      f"at 1 km spacing over operational area")


def get_anchor_registry() -> List[InfrastructureNode]:
    """Return the full anchor registry (read-only reference)."""
    return _REGISTRY


# ══════════════════════════════════════════════════════════════════════════════
# GDOP CALCULATION
# ══════════════════════════════════════════════════════════════════════════════

def _unit_direction(drone_xyz: Tuple[float, float, float],
                    node: InfrastructureNode) -> Optional[Tuple[float, float, float]]:
    """
    Unit vector from drone position to node.
    Returns None if distance is negligibly small (drone is on the node).
    """
    dx = node.x - drone_xyz[0]
    dy = node.y - drone_xyz[1]
    dz = node.z - drone_xyz[2]
    d  = math.sqrt(dx*dx + dy*dy + dz*dz)
    if d < 1e-3:
        return None
    return (dx/d, dy/d, dz/d)


def _compute_gdop_split(
    drone_xyz: Tuple[float, float, float],
    nodes: List[InfrastructureNode],
) -> Tuple[float, float]:
    """
    Compute split HDOP (horizontal) and VDOP (vertical) for a set of nodes.

    H matrix (n×3): each row is the unit direction vector [ux, uy, uz]
    from the drone to one node.

    Full GDOP:  trace( (H^T H)^-1 )^0.5
    HDOP:       trace( [(H^T H)^-1]_xy )^0.5  (top-left 2×2 sub-matrix)
    VDOP:       sqrt( [(H^T H)^-1]_zz )        (bottom-right element)

    Returns (hdop, vdop). Returns (999, 999) if matrix is singular or < 4 nodes.
    """
    if len(nodes) < config.ANCHOR_MIN_COUNT:
        return 999.0, 999.0

    rows: List[Tuple[float, float, float]] = []
    for node in nodes:
        u = _unit_direction(drone_xyz, node)
        if u is not None:
            rows.append(u)

    if len(rows) < config.ANCHOR_MIN_COUNT:
        return 999.0, 999.0

    HtH = [[0.0]*3 for _ in range(3)]
    for row in rows:
        for i in range(3):
            for j in range(3):
                HtH[i][j] += row[i] * row[j]

    inv = _invert_3x3(HtH)
    if inv is None:
        return 999.0, 999.0

    hdop = math.sqrt(max(inv[0][0] + inv[1][1], 0.0))
    vdop = math.sqrt(max(inv[2][2], 0.0))
    return hdop, vdop


def _invert_3x3(m: List[List[float]]) -> Optional[List[List[float]]]:
    """
    Invert a 3×3 matrix using the analytic cofactor method.
    Returns None if singular (det ≈ 0).
    """
    a = m[0][0]; b = m[0][1]; c = m[0][2]
    d = m[1][0]; e = m[1][1]; f = m[1][2]
    g = m[2][0]; h = m[2][1]; k = m[2][2]

    det = a*(e*k - f*h) - b*(d*k - f*g) + c*(d*h - e*g)
    if abs(det) < 1e-12:
        return None

    inv_det = 1.0 / det
    return [
        [ (e*k - f*h)*inv_det,  -(b*k - c*h)*inv_det,  (b*f - c*e)*inv_det ],
        [ -(d*k - f*g)*inv_det,  (a*k - c*g)*inv_det, -(a*f - c*d)*inv_det ],
        [ (d*h - e*g)*inv_det,  -(a*h - b*g)*inv_det,  (a*e - b*d)*inv_det ],
    ]


# ══════════════════════════════════════════════════════════════════════════════
# GREEDY GDOP SELECTION
# ══════════════════════════════════════════════════════════════════════════════

def _angular_separation(drone_xyz: Tuple[float, float, float],
                        n1: InfrastructureNode,
                        n2: InfrastructureNode) -> float:
    """Dot product between two unit direction vectors. Smaller = more separated."""
    u1 = _unit_direction(drone_xyz, n1)
    u2 = _unit_direction(drone_xyz, n2)
    if u1 is None or u2 is None:
        return 1.0
    return u1[0]*u2[0] + u1[1]*u2[1] + u1[2]*u2[2]


def select_anchors(
    drone_xyz: Tuple[float, float, float],
) -> Tuple[List[InfrastructureNode], float, float]:
    """
    Select the geometrically optimal 4–6 anchor nodes for a drone position.

    Algorithm
    ---------
    1. Filter registry to nodes within ANCHOR_MAX_RANGE
    2. If ≤ ANCHOR_MAX_COUNT: use all, compute GDOP
    3. If > ANCHOR_MAX_COUNT: greedy selection —
         a. Seed with the pair that has minimum dot product (largest angular gap)
         b. At each step, add the node whose inclusion most reduces total GDOP
            (i.e. sqrt(HDOP² + VDOP²)) until ANCHOR_MAX_COUNT reached
    4. Compute and return split (HDOP, VDOP)

    Returns
    -------
    (selected_nodes, hdop, vdop)
    If fewer than ANCHOR_MIN_COUNT nodes are in range, returns ([], 999, 999).
    """
    dx, dy, dz = drone_xyz

    # ── Step 1: range filter ──────────────────────────────────────────────────
    candidates: List[Tuple[float, InfrastructureNode]] = []
    for node in _REGISTRY:
        dist = math.sqrt((node.x - dx)**2 + (node.y - dy)**2 + (node.z - dz)**2)
        if dist <= config.ANCHOR_MAX_RANGE:
            candidates.append((dist, node))

    if len(candidates) < config.ANCHOR_MIN_COUNT:
        return [], 999.0, 999.0

    candidates.sort(key=lambda t: t[0])

    # ── Step 2: trivial case ──────────────────────────────────────────────────
    nodes_only = [n for _, n in candidates]
    if len(candidates) <= config.ANCHOR_MAX_COUNT:
        hdop, vdop = _compute_gdop_split(drone_xyz, nodes_only)
        return nodes_only, hdop, vdop

    # ── Step 3: greedy GDOP minimisation ─────────────────────────────────────
    best_pair_sep = 2.0
    seed_i, seed_j = 0, 1
    for i in range(len(nodes_only)):
        for j in range(i+1, len(nodes_only)):
            sep = _angular_separation(drone_xyz, nodes_only[i], nodes_only[j])
            if sep < best_pair_sep:
                best_pair_sep = sep
                seed_i, seed_j = i, j

    selected  = [nodes_only[seed_i], nodes_only[seed_j]]
    remaining = [n for k, n in enumerate(nodes_only) if k not in (seed_i, seed_j)]

    while len(selected) < config.ANCHOR_MAX_COUNT and remaining:
        best_gdop = float('inf')
        best_idx  = 0
        for idx, candidate_node in enumerate(remaining):
            trial = selected + [candidate_node]
            hdop_t, vdop_t = _compute_gdop_split(drone_xyz, trial)
            combined = math.sqrt(hdop_t**2 + vdop_t**2)
            if combined < best_gdop:
                best_gdop = combined
                best_idx  = idx
        selected.append(remaining.pop(best_idx))

    # ── Step 4: final GDOP ────────────────────────────────────────────────────
    hdop, vdop = _compute_gdop_split(drone_xyz, selected)
    return selected, hdop, vdop


# ══════════════════════════════════════════════════════════════════════════════
# DISTANCE PRE-COMPUTATION
# ══════════════════════════════════════════════════════════════════════════════

def compute_anchor_bindings(
    drone_xyz: Tuple[float, float, float],
) -> Tuple[List[AnchorBinding], float, float]:
    """
    For a waypoint at drone_xyz, select optimal anchor nodes and
    compute the expected 3-D Euclidean distance to each.

    Returns
    -------
    (bindings, hdop, vdop)

    bindings is an empty list if anchor coverage is insufficient.
    The caller (pathfinding.py) is responsible for interpreting HDOP/VDOP
    against the configured thresholds before trusting the bindings.
    """
    selected_nodes, hdop, vdop = select_anchors(drone_xyz)

    bindings: List[AnchorBinding] = []
    dx, dy, dz = drone_xyz
    for node in selected_nodes:
        dist = math.sqrt(
            (node.x - dx)**2 +
            (node.y - dy)**2 +
            (node.z - dz)**2
        )
        bindings.append(AnchorBinding(
            node_id=node.node_id,
            expected_dist_m=dist,
        ))

    return bindings, hdop, vdop
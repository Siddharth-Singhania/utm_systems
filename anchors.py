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
# ANCHOR REGISTRY  (25 infrastructure nodes across SF)
# ══════════════════════════════════════════════════════════════════════════════
#
# Node placement strategy:
#   - Spread across the operational area (37.50–37.90 lat, -122.55–-122.25 lon)
#   - Mounted on city infrastructure (civic buildings, transit hubs, towers)
#   - Altitudes reflect realistic rooftop / tower heights
#   - Geographic distribution ensures no > 2 km dead zones in the coverage grid
#
_REGISTRY: List[InfrastructureNode] = [
    # ── Civic Core ───────────────────────────────────────────────────────────
    InfrastructureNode("N01", "CivicCenter_Tower",       37.7793, -122.4193, 55.0),
    InfrastructureNode("N02", "UnionSquare_Roof",        37.7879, -122.4075, 48.0),
    InfrastructureNode("N03", "TransbayTerminal",        37.7895, -122.3969, 62.0),
    InfrastructureNode("N04", "FinancialDistrict_Tower", 37.7946, -122.3998, 78.0),

    # ── Northern SF ──────────────────────────────────────────────────────────
    InfrastructureNode("N05", "FishermansWharf_Mast",    37.8080, -122.4177, 38.0),
    InfrastructureNode("N06", "CrisseField_Beacon",      37.8034, -122.4654, 22.0),
    InfrastructureNode("N07", "NorthBeach_Tower",        37.8009, -122.4103, 42.0),
    InfrastructureNode("N08", "AlcatrazLanding",         37.8270, -122.4230, 28.0),

    # ── Western SF ───────────────────────────────────────────────────────────
    InfrastructureNode("N09", "GGPark_WaterTower",       37.7694, -122.4862, 35.0),
    InfrastructureNode("N10", "SutroTower_Relay",        37.7552, -122.4528, 298.0),  # tall — excellent VDOP anchor
    InfrastructureNode("N11", "OuterSunset_Relay",       37.7600, -122.5050, 30.0),
    InfrastructureNode("N12", "OceanBeach_Mast",         37.7290, -122.5060, 25.0),

    # ── Central SF ───────────────────────────────────────────────────────────
    InfrastructureNode("N13", "Castro_CommHub",          37.7609, -122.4350, 44.0),
    InfrastructureNode("N14", "Mission_MidHub",          37.7599, -122.4148, 38.0),
    InfrastructureNode("N15", "Potrero_Rooftop",         37.7560, -122.4000, 32.0),
    InfrastructureNode("N16", "DogpatchYard",            37.7570, -122.3880, 28.0),

    # ── Eastern SF / Bay Edge ────────────────────────────────────────────────
    InfrastructureNode("N17", "BayBridge_WTower",        37.7954, -122.3770, 155.0),  # bridge tower — superb geometry
    InfrastructureNode("N18", "India_Basin_Beacon",      37.7390, -122.3760, 22.0),
    InfrastructureNode("N19", "Hunters_Point_Relay",     37.7238, -122.3760, 30.0),

    # ── Southern SF / Peninsula ──────────────────────────────────────────────
    InfrastructureNode("N20", "GlenPark_Hub",            37.7329, -122.4337, 38.0),
    InfrastructureNode("N21", "ExcelsiorRelay",          37.7210, -122.4280, 34.0),
    InfrastructureNode("N22", "DalyCity_North",          37.6880, -122.4702, 42.0),
    InfrastructureNode("N23", "SFO_North_Tower",         37.6275, -122.3790, 18.0),

    # ── Far South (extends to SFO-adjacent coverage) ─────────────────────────
    InfrastructureNode("N24", "BrisbaneHill_Relay",      37.6060, -122.4050, 55.0),
    InfrastructureNode("N25", "SouthSF_CivicMast",       37.6540, -122.4080, 40.0),
]


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

    # Build H matrix rows
    rows: List[Tuple[float, float, float]] = []
    for node in nodes:
        u = _unit_direction(drone_xyz, node)
        if u is not None:
            rows.append(u)

    if len(rows) < config.ANCHOR_MIN_COUNT:
        return 999.0, 999.0

    # H^T H  — 3×3 symmetric matrix, computed manually for zero dependencies
    # H^T H[i][j] = sum_k( rows[k][i] * rows[k][j] )
    HtH = [[0.0]*3 for _ in range(3)]
    for row in rows:
        for i in range(3):
            for j in range(3):
                HtH[i][j] += row[i] * row[j]

    # Invert 3×3 matrix using cofactor expansion
    inv = _invert_3x3(HtH)
    if inv is None:
        return 999.0, 999.0

    # HDOP = sqrt(inv[0][0] + inv[1][1])  — horizontal plane variance
    # VDOP = sqrt(inv[2][2])              — vertical variance
    hdop_sq = inv[0][0] + inv[1][1]
    vdop_sq = inv[2][2]

    hdop = math.sqrt(max(hdop_sq, 0.0))
    vdop = math.sqrt(max(vdop_sq, 0.0))
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

    candidates.sort(key=lambda t: t[0])   # nearest first

    # ── Step 2: trivial case ──────────────────────────────────────────────────
    nodes_only = [n for _, n in candidates]
    if len(candidates) <= config.ANCHOR_MAX_COUNT:
        hdop, vdop = _compute_gdop_split(drone_xyz, nodes_only)
        return nodes_only, hdop, vdop

    # ── Step 3: greedy GDOP minimisation ─────────────────────────────────────
    # Seed: the pair with the largest angular separation
    best_pair_sep = 2.0   # dot product starts at max = 1.0, we want minimum
    seed_i, seed_j = 0, 1
    for i in range(len(nodes_only)):
        for j in range(i+1, len(nodes_only)):
            sep = _angular_separation(drone_xyz, nodes_only[i], nodes_only[j])
            if sep < best_pair_sep:
                best_pair_sep = sep
                seed_i, seed_j = i, j

    selected   = [nodes_only[seed_i], nodes_only[seed_j]]
    remaining  = [n for k, n in enumerate(nodes_only) if k not in (seed_i, seed_j)]

    while len(selected) < config.ANCHOR_MAX_COUNT and remaining:
        best_gdop   = float('inf')
        best_idx    = 0

        for idx, candidate_node in enumerate(remaining):
            trial = selected + [candidate_node]
            hdop_t, vdop_t = _compute_gdop_split(drone_xyz, trial)
            combined_gdop  = math.sqrt(hdop_t**2 + vdop_t**2)
            if combined_gdop < best_gdop:
                best_gdop = combined_gdop
                best_idx  = idx

        selected.append(remaining.pop(best_idx))

    # ── Step 4: final GDOP ────────────────────────────────────────────────────
    hdop, vdop = _compute_gdop_split(drone_xyz, selected)
    return selected, hdop, vdop


# ══════════════════════════════════════════════════════════════════════════════
# DISTANCE PRE-COMPUTATION  (called from pathfinding.build_4d_trajectory)
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
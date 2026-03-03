"""
gps_denied.py — GPS-Denied Navigation: WLS Position Correction
================================================================

Public API
----------
simulate_ranging(bindings, true_xyz)
    -> List[Tuple[str, float, float]]
        Synthesise noisy UWB range measurements for a drone's true position.
        Returns [(node_id, expected_dist, measured_dist), ...]

compute_wls_correction(bindings, measurements, drone_xyz)
    -> WLSResult

WLSResult
    .delta_xyz      (Δx, Δy, Δz) metres — position correction vector
    .velocity_correction (vx, vy, vz) m/s — delta_xyz / tau, ready to apply
    .hdop_ok        bool — horizontal geometry was sufficient
    .vdop_ok        bool — vertical geometry was sufficient
    .n_nodes        int  — number of nodes used in solution
    .residual_rms   float — RMS of ranging residuals after correction (quality metric)

Math overview
-------------
If the drone has drifted by Δp = (Δx, Δy, Δz) from its planned position,
the change in range to anchor i is approximately:

    Δdᵢ ≈ −ûᵢ · Δp

where ûᵢ is the unit vector from PLANNED position to anchor i.
(Negative: moving toward a node decreases distance.)

Stacking n equations:
    H · Δp = −r

    H   (n×3)  — unit direction matrix
    Δp  (3×1)  — unknown displacement
    r   (n×1)  — residual vector:  rᵢ = measured_i − expected_i

Weighted Least Squares (WLS) solution:
    Δp = −(HᵀWH)⁻¹ · Hᵀ · W · r

Weight matrix W (diagonal):
    wᵢ = 1 / σᵢ²
    σᵢ = UWB_SIGMA_0 + UWB_DRIFT_K × expected_dist_i

Closer nodes → smaller σ → higher weight. Natural distance-based de-weighting.

HDOP / VDOP gating
-------------------
Before solving, HDOP and VDOP are computed from H.
If HDOP > HDOP_THRESHOLD:  Δx = Δy = 0  (dead reckoning in horizontal plane)
If VDOP > VDOP_THRESHOLD:  Δz = 0       (barometric altimeter maintains altitude)

Velocity correction
-------------------
    v_correction = Δp / τ   (τ = WLS_CORRECTION_TAU seconds)

Added to the nominal velocity vector for one correction window.
"""

import math
import random
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import config
from models import AnchorBinding
from anchors import InfrastructureNode, get_anchor_registry, _invert_3x3


# ── Shared node lookup  (built once at module load) ───────────────────────────
_NODE_MAP: Dict[str, InfrastructureNode] = {
    n.node_id: n for n in get_anchor_registry()
}


# ══════════════════════════════════════════════════════════════════════════════
# RESULT DATA CLASS
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class WLSResult:
    """
    Output of compute_wls_correction().

    All displacement values are in metres in the local Cartesian frame.
    velocity_correction is in m/s — add to the drone's nominal velocity
    for one WLS_CORRECTION_TAU-second window.
    """
    delta_xyz:            Tuple[float, float, float]   # (Δx, Δy, Δz) metres
    velocity_correction:  Tuple[float, float, float]   # (vx, vy, vz) m/s
    hdop_ok:              bool    # True if horizontal geometry was adequate
    vdop_ok:              bool    # True if vertical geometry was adequate
    n_nodes:              int     # number of anchor nodes used
    residual_rms:         float   # RMS ranging residual after correction (m)


# ══════════════════════════════════════════════════════════════════════════════
# RANGING SIMULATION  (synthesises UWB measurements with Gaussian noise)
# ══════════════════════════════════════════════════════════════════════════════

def simulate_ranging(
    bindings:  List[AnchorBinding],
    true_xyz:  Tuple[float, float, float],
) -> List[Tuple[str, float, float]]:
    """
    Synthesise noisy UWB range measurements for a drone at true_xyz.

    For each anchor binding we know the EXPECTED distance (from the flight plan
    at the planned position). We compute the TRUE distance from the ACTUAL
    position and add Gaussian noise scaled by that true distance:

        σᵢ = UWB_SIGMA_0 + UWB_DRIFT_K × true_dist_i
        measured_i = true_dist_i + N(0, σᵢ)

    Returns a list of (node_id, expected_dist_m, measured_dist_m) tuples.
    Nodes not found in the registry (stale plan) are silently skipped.
    """
    results: List[Tuple[str, float, float]] = []
    tx, ty, tz = true_xyz

    for binding in bindings:
        node = _NODE_MAP.get(binding.node_id)
        if node is None:
            continue

        # True Euclidean distance from the actual (possibly drifted) position
        true_dist = math.sqrt(
            (node.x - tx)**2 +
            (node.y - ty)**2 +
            (node.z - tz)**2
        )

        # Distance-dependent Gaussian noise model
        sigma = config.UWB_SIGMA_0 + config.UWB_DRIFT_K * true_dist
        noise = random.gauss(0.0, sigma)
        measured = max(0.1, true_dist + noise)   # distance must be positive

        results.append((binding.node_id, binding.expected_dist_m, measured))

    return results


# ══════════════════════════════════════════════════════════════════════════════
# WLS CORRECTION SOLVER
# ══════════════════════════════════════════════════════════════════════════════

def _build_H_and_residuals(
    bindings:     List[AnchorBinding],
    measurements: List[Tuple[str, float, float]],   # (node_id, expected, measured)
    planned_xyz:  Tuple[float, float, float],
) -> Tuple[
    List[Tuple[float, float, float]],   # H rows (unit direction vectors)
    List[float],                         # residuals r
    List[float],                         # weights w
]:
    """
    Build the H matrix rows, residual vector, and weight vector.

    Unit direction ûᵢ points FROM planned drone position TO anchor node.
    Residual rᵢ = measured_i − expected_i
    Weight    wᵢ = 1 / σᵢ²  where σᵢ = UWB_SIGMA_0 + UWB_DRIFT_K × expected_i
    """
    meas_map: Dict[str, Tuple[float, float]] = {
        node_id: (expected, measured)
        for node_id, expected, measured in measurements
    }

    H_rows:   List[Tuple[float, float, float]] = []
    residuals: List[float]                      = []
    weights:   List[float]                      = []

    px, py, pz = planned_xyz

    for binding in bindings:
        node = _NODE_MAP.get(binding.node_id)
        if node is None:
            continue
        meas_entry = meas_map.get(binding.node_id)
        if meas_entry is None:
            continue

        expected, measured = meas_entry

        # Unit direction vector from planned position to node
        dx = node.x - px
        dy = node.y - py
        dz = node.z - pz
        d  = math.sqrt(dx*dx + dy*dy + dz*dz)
        if d < 1e-3:
            continue

        ux, uy, uz = dx/d, dy/d, dz/d
        H_rows.append((ux, uy, uz))

        # rᵢ = measured − expected
        residuals.append(measured - expected)

        # wᵢ = 1 / σᵢ²
        sigma_i = config.UWB_SIGMA_0 + config.UWB_DRIFT_K * expected
        weights.append(1.0 / max(sigma_i**2, 1e-9))

    return H_rows, residuals, weights


def _gdop_from_H(
    H_rows: List[Tuple[float, float, float]],
) -> Tuple[float, float]:
    """
    Compute HDOP and VDOP from H matrix rows.
    Mirrors the logic in anchors._compute_gdop_split but operates on
    pre-built rows rather than InfrastructureNode objects.
    """
    if len(H_rows) < config.ANCHOR_MIN_COUNT:
        return 999.0, 999.0

    HtH = [[0.0]*3 for _ in range(3)]
    for row in H_rows:
        for i in range(3):
            for j in range(3):
                HtH[i][j] += row[i] * row[j]

    inv = _invert_3x3(HtH)
    if inv is None:
        return 999.0, 999.0

    hdop = math.sqrt(max(inv[0][0] + inv[1][1], 0.0))
    vdop = math.sqrt(max(inv[2][2], 0.0))
    return hdop, vdop


def _solve_wls(
    H_rows:    List[Tuple[float, float, float]],
    residuals: List[float],
    weights:   List[float],
) -> Optional[Tuple[float, float, float]]:
    """
    Solve the weighted least squares system:
        Δp = −(HᵀWH)⁻¹ · Hᵀ · W · r

    Returns (Δx, Δy, Δz) in metres, or None if the system is singular.

    The negative sign comes from the linearisation Δdᵢ ≈ −ûᵢ·Δp,
    so H·Δp = −r, therefore Δp = −(HᵀWH)⁻¹·Hᵀ·W·r.
    """
    n = len(H_rows)
    if n < config.ANCHOR_MIN_COUNT:
        return None

    # ── HᵀWH  (3×3) ──────────────────────────────────────────────────────────
    # HᵀWH[i][j] = Σₖ wₖ · H[k][i] · H[k][j]
    HtWH = [[0.0]*3 for _ in range(3)]
    for k in range(n):
        w  = weights[k]
        row = H_rows[k]
        for i in range(3):
            for j in range(3):
                HtWH[i][j] += w * row[i] * row[j]

    # ── (HᵀWH)⁻¹ ─────────────────────────────────────────────────────────────
    inv = _invert_3x3(HtWH)
    if inv is None:
        return None

    # ── Hᵀ W r  (3×1) ────────────────────────────────────────────────────────
    # HtWr[i] = Σₖ wₖ · H[k][i] · r[k]
    HtWr = [0.0, 0.0, 0.0]
    for k in range(n):
        w = weights[k]
        r = residuals[k]
        for i in range(3):
            HtWr[i] += w * H_rows[k][i] * r

    # ── Δp = −(HᵀWH)⁻¹ · Hᵀ W r ─────────────────────────────────────────────
    delta = [0.0, 0.0, 0.0]
    for i in range(3):
        for j in range(3):
            delta[i] -= inv[i][j] * HtWr[j]

    return (delta[0], delta[1], delta[2])


def _compute_residual_rms(
    H_rows:    List[Tuple[float, float, float]],
    residuals: List[float],
    delta_xyz: Tuple[float, float, float],
) -> float:
    """
    Compute RMS of ranging residuals AFTER applying the correction.
    The post-correction residual for node i is:
        eᵢ = rᵢ + ûᵢ · Δp   (note: H·Δp ≈ −r, so eᵢ → 0 for a perfect fix)
    Lower RMS = better fix quality.
    """
    if not residuals:
        return 0.0

    dx, dy, dz = delta_xyz
    sse = 0.0
    for k, (ux, uy, uz) in enumerate(H_rows):
        predicted_delta_d = -(ux*dx + uy*dy + uz*dz)
        post_residual = residuals[k] - predicted_delta_d
        sse += post_residual**2
    return math.sqrt(sse / len(residuals))


# ══════════════════════════════════════════════════════════════════════════════
# PUBLIC ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════

def compute_wls_correction(
    bindings:     List[AnchorBinding],
    measurements: List[Tuple[str, float, float]],
    planned_xyz:  Tuple[float, float, float],
) -> WLSResult:
    """
    Compute a WLS position correction from anchor range measurements.

    Parameters
    ----------
    bindings     : AnchorBinding list from the current Waypoint4D
    measurements : output of simulate_ranging() —
                   list of (node_id, expected_dist_m, measured_dist_m)
    planned_xyz  : the drone's planned (x, y, z) in local Cartesian metres

    Returns
    -------
    WLSResult with delta_xyz, velocity_correction, HDOP/VDOP flags,
    node count, and post-correction residual RMS.

    HDOP / VDOP gating
    ------------------
    If HDOP > HDOP_THRESHOLD: Δx = Δy = 0 (horizontal dead reckoning)
    If VDOP > VDOP_THRESHOLD: Δz = 0      (barometric altitude hold)

    If fewer than ANCHOR_MIN_COUNT nodes are available after filtering,
    delta_xyz = (0,0,0) and hdop_ok = vdop_ok = False (full dead reckoning).
    """
    H_rows, residuals, weights = _build_H_and_residuals(
        bindings, measurements, planned_xyz
    )

    # ── GDOP gating ───────────────────────────────────────────────────────────
    hdop, vdop = _gdop_from_H(H_rows)
    hdop_ok = hdop <= config.HDOP_THRESHOLD
    vdop_ok = vdop <= config.VDOP_THRESHOLD

    # ── Solve WLS ─────────────────────────────────────────────────────────────
    raw_delta = _solve_wls(H_rows, residuals, weights)

    if raw_delta is None or (not hdop_ok and not vdop_ok):
        # Full dead reckoning — no correction applied
        zero = (0.0, 0.0, 0.0)
        return WLSResult(
            delta_xyz=zero,
            velocity_correction=zero,
            hdop_ok=hdop_ok,
            vdop_ok=vdop_ok,
            n_nodes=len(H_rows),
            residual_rms=0.0,
        )

    dx, dy, dz = raw_delta

    # ── Apply HDOP / VDOP gates ───────────────────────────────────────────────
    if not hdop_ok:
        dx = 0.0
        dy = 0.0   # horizontal dead reckoning
    if not vdop_ok:
        dz = 0.0   # barometric altitude hold

    delta_xyz = (dx, dy, dz)

    # ── Velocity correction: Δp distributed over τ seconds ───────────────────
    tau = config.WLS_CORRECTION_TAU
    velocity_correction = (dx / tau, dy / tau, dz / tau)

    # ── Post-correction residual RMS ──────────────────────────────────────────
    rms = _compute_residual_rms(H_rows, residuals, delta_xyz)

    return WLSResult(
        delta_xyz=delta_xyz,
        velocity_correction=velocity_correction,
        hdop_ok=hdop_ok,
        vdop_ok=vdop_ok,
        n_nodes=len(H_rows),
        residual_rms=rms,
    )
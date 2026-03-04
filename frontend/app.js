// ============================================================
// UTM SYSTEM — FRONTEND
// - No drones visible until a mission is assigned
// - Routes hidden by default; toggle shows all
// - Clicking a drone shows its route + live speed/alt panel
// ============================================================

const API_URL = window.location.origin;
const WS_URL  = `${window.location.protocol === 'https:' ? 'wss:' : 'ws:'}//${window.location.host}/ws`;
const MIN_ALTITUDE = 20.0;

// ── Infrastructure Node Registry ─────────────────────────────
// Populated dynamically from the backend's initial_state WebSocket message.
// Keyed by node_id (e.g. "N01") → { label, lat, lon, alt }
// Coordinates live only in anchors.py — this is the single source of truth.
let INFRASTRUCTURE_NODES = {};

const DEMO_SCENARIOS = {
    // ── Standard routes ───────────────────────────────────────────────────────
    1:  { label: "Downtown → W.Portal",           distance: "~9km",  pickup: { lat: 37.7749, lon: -122.4194 }, delivery: { lat: 37.6879, lon: -122.4702 } },
    2:  { label: "Marina → Inner Richmond",        distance: "~4km",  pickup: { lat: 37.8055, lon: -122.4367 }, delivery: { lat: 37.7946, lon: -122.3999 } },
    3:  { label: "Mission → North Beach",          distance: "~6km",  pickup: { lat: 37.7599, lon: -122.4148 }, delivery: { lat: 37.8009, lon: -122.4103 } },
    4:  { label: "GG Park → North Beach",          distance: "~10km", pickup: { lat: 37.7694, lon: -122.4862 }, delivery: { lat: 37.7955, lon: -122.3937 } },
    5:  { label: "Castro → Fishermans Wharf",      distance: "~7km",  pickup: { lat: 37.7609, lon: -122.4350 }, delivery: { lat: 37.8080, lon: -122.4177 } },

    // ── Reroute demos (straight-line crosses a no-fly zone) ───────────────────
    6:  { label: "SFO North → Brisbane",           distance: "~9km",  reroute: true, zone: "Airport",  pickup: { lat: 37.6600, lon: -122.3700 }, delivery: { lat: 37.5800, lon: -122.3700 } },
    7:  { label: "Noe Valley → San Bruno",         distance: "~11km", reroute: true, zone: "Airport",  pickup: { lat: 37.6400, lon: -122.4200 }, delivery: { lat: 37.5800, lon: -122.3550 } },
    8:  { label: "Inner Richmond → Outer Sunset",  distance: "~6km",  reroute: true, zone: "Presidio", pickup: { lat: 37.8000, lon: -122.4400 }, delivery: { lat: 37.7750, lon: -122.5030 } },
    9:  { label: "Westlake → Bayshore",            distance: "~7km",  reroute: true, zone: "Airport",  pickup: { lat: 37.6050, lon: -122.4200 }, delivery: { lat: 37.6050, lon: -122.3400 } },
    10: { label: "South SF → Daly City",           distance: "~6km",  reroute: true, zone: "Airport",  pickup: { lat: 37.5900, lon: -122.3700 }, delivery: { lat: 37.6400, lon: -122.3700 } },
    11: { label: "Pacific Heights → Outer Sunset", distance: "~9km",  reroute: true, zone: "Presidio", pickup: { lat: 37.8050, lon: -122.4300 }, delivery: { lat: 37.7600, lon: -122.5050 } },

    // ── Conflict demos ────────────────────────────────────────────────────────
    // Each entry is a PAIR launched back-to-back (<600 ms apart).
    // Expected resolution is noted so the operator knows what to look for.
    // Launch both legs of a pair via loadConflictScenario(id).

    // 12a/12b — Counter-flow: same corridor, opposite directions.
    // NORTH drone is assigned stratum 60 m, SOUTH drone 40 m.
    // With pad queue off both launch simultaneously → their 4D paths overlap
    // in the cruise segment → resolver switches one to the next free stratum.
    // Expected: ALTITUDE CHANGE alert (e.g. 60 m → 80 m).
    12: { label: "Downtown ↔ Mission (counter-flow)", conflict: "altitude", pair: [
        { pickup: { lat: 37.7749, lon: -122.4194 }, delivery: { lat: 37.7599, lon: -122.4148 } },
        { pickup: { lat: 37.7599, lon: -122.4148 }, delivery: { lat: 37.7749, lon: -122.4194 } },
    ]},

    // 13a/13b — Counter-flow: longer north–south corridor.
    // Expected: ALTITUDE CHANGE alert.
    13: { label: "NorthBeach ↔ Castro (counter-flow)", conflict: "altitude", pair: [
        { pickup: { lat: 37.8009, lon: -122.4103 }, delivery: { lat: 37.7609, lon: -122.4350 } },
        { pickup: { lat: 37.7609, lon: -122.4350 }, delivery: { lat: 37.8009, lon: -122.4103 } },
    ]},

    // 14a/14b — Same route, same direction, rapid succession.
    // Both drones follow identical road paths → CPA detects head-on overlap
    // in time even though directions match.
    // Expected: TAKEOFF DELAY alert (binary-search delay applied to drone 2).
    14: { label: "Marina → Potrero (same path ×2)", conflict: "delay", pair: [
        { pickup: { lat: 37.8055, lon: -122.4367 }, delivery: { lat: 37.7560, lon: -122.4000 } },
        { pickup: { lat: 37.8055, lon: -122.4367 }, delivery: { lat: 37.7560, lon: -122.4000 } },
    ]},

    // 15a/15b — Intersecting diagonals.
    // SW→NE crosses NW→SE roughly over the Civic Center area.
    // Expected: ALTITUDE CHANGE alert (different strata assigned to each diagonal).
    15: { label: "GGPark → TransBay ✕ Marina → GlenPark (crossing)", conflict: "altitude", pair: [
        { pickup: { lat: 37.7694, lon: -122.4862 }, delivery: { lat: 37.7895, lon: -122.3969 } },
        { pickup: { lat: 37.8055, lon: -122.4367 }, delivery: { lat: 37.7329, lon: -122.4337 } },
    ]},

    // 16a/16b — East–West counter-flow.
    // EAST drone stratum 50 m, WEST drone stratum 30 m (20 m gap).
    // Overlap forces resolver to switch one to 70 m or apply delay.
    // Expected: ALTITUDE CHANGE or TAKEOFF DELAY alert.
    16: { label: "Sunset ↔ Potrero (E–W counter-flow)", conflict: "altitude", pair: [
        { pickup: { lat: 37.7560, lon: -122.4862 }, delivery: { lat: 37.7560, lon: -122.4000 } },
        { pickup: { lat: 37.7560, lon: -122.4000 }, delivery: { lat: 37.7560, lon: -122.4862 } },
    ]},
};



// ── Cesium ───────────────────────────────────────────────────
Cesium.Ion.defaultAccessToken = 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJqdGkiOiI1ZWQ4NmZjZS1hMGVkLTQzNjYtYTIwZi1kYWFmMDM2NmYzZTIiLCJpZCI6MzkwMDE1LCJpYXQiOjE3NzA5MDQ5MTJ9.oMikqljPMbCSIgEWEdCwZZnJyFDDLDU4RUYYpUrXxMI';

const viewer = new Cesium.Viewer('cesiumContainer', {
    terrain: Cesium.Terrain.fromWorldTerrain(),
    baseLayerPicker: false, geocoder: false, homeButton: false,
    sceneModePicker: false, navigationHelpButton: false,
    animation: false, timeline: false, fullscreenButton: false,
    vrButton: false, infoBox: false, selectionIndicator: false
});

viewer.clock.shouldAnimate  = true;
viewer.clock.multiplier     = 1.0;
viewer.clock.clockStep      = Cesium.ClockStep.SYSTEM_CLOCK_MULTIPLIER;
setInterval(() => { viewer.clock.currentTime = Cesium.JulianDate.now(); }, 100);

viewer.camera.setView({
    destination: Cesium.Cartesian3.fromDegrees(-122.4, 37.7, 20000),
    orientation: { heading: Cesium.Math.toRadians(0), pitch: Cesium.Math.toRadians(-45), roll: 0 }
});

// ── State ────────────────────────────────────────────────────
// missions map:  missionId → { droneId, droneEntity, routeEntity, pickupMarker, deliveryMarker, waypoints, positionProperty }
const missions       = new Map();
// latestTelemetry:  droneId → telemetry object (updated from WS)
const latestTelemetry = new Map();
// droneIdToMission:  droneId → missionId  (for click lookup)
const droneIdToMission = new Map();

let showAllRoutes        = false;  // toggle: show all routes at once
let showRerouteAnimation = true;   // toggle: animate blocked→rerouted path
let showUWBRanging       = false;  // toggle: show UWB ranging lines globally
let selectedMission = null;    // currently selected missionId
let infoUpdateTimer = null;
let ws = null;

// ── UWB ranging state ─────────────────────────────────────────
// droneId → { lat, lon, alt, nodeIds[] }  (last known telemetry for redraw)
const lastRangingData = new Map();

// ── PolylineCollection for UWB ranging lines ───────────────────
// Using scene primitives instead of entities because:
//   • Entity polylines are depth-tested against terrain (clipped underground)
//   • PolylineCollection gives direct show/positions control per line
//   • No entity ID collision issues
const _uwbLineCollection = new Cesium.PolylineCollection();
viewer.scene.primitives.add(_uwbLineCollection);

// droneId → nodeId → Cesium.Polyline object
const _droneNodeLines = new Map();

const _UWB_MATERIAL = Cesium.Material.fromType('PolylineDash', {
    color: new Cesium.Color(0.065, 0.725, 0.506, 0.9),   // #10b981 green
    dashLength: 15.0,
    dashPattern: 255.0
});
const _UWB_MATERIAL_SEL = Cesium.Material.fromType('PolylineDash', {
    color: new Cesium.Color(0.204, 0.855, 0.600, 1.0),   // Bright green
    dashLength: 15.0,
    dashPattern: 255.0
});

// depthFail material renders lines that pass through terrain (same color, lower alpha)
const _UWB_DEPTH_FAIL = Cesium.Material.fromType('Color', {
    color: new Cesium.Color(0.065, 0.725, 0.506, 0.3)
});

// ── WebSocket ────────────────────────────────────────────────
function connectWebSocket() {
    ws = new WebSocket(WS_URL);
    ws.onopen    = () => { console.log('WS connected'); document.getElementById('loading').style.display = 'none'; };
    ws.onmessage = (e) => handleWebSocketMessage(JSON.parse(e.data));
    ws.onerror   = (e) => console.error('WS error:', e);
    ws.onclose   = () => setTimeout(connectWebSocket, 3000);
}

function handleWebSocketMessage(data) {
    switch (data.type) {
        case 'initial_state':
            // Populate anchor registry from backend — anchors.py is the single
            // source of truth. initAnchorTowers() is deferred to here so towers
            // are always drawn at the authoritative coordinates.
            if (data.anchor_nodes && data.anchor_nodes.length > 0) {
                INFRASTRUCTURE_NODES = {};
                data.anchor_nodes.forEach(n => {
                    INFRASTRUCTURE_NODES[n.node_id] = {
                        label: n.label, lat: n.lat, lon: n.lon, alt: n.alt
                    };
                });
                console.log('[UWB] Loaded ' + data.anchor_nodes.length + ' anchor nodes from backend');
                initAnchorTowers();
            } else {
                console.warn('[UWB] initial_state had no anchor_nodes');
            }
            loadGeofencing(data.geofencing);
            break;
        case 'mission_created':
            spawnMissionDrone(data.mission, data.blocked_path || null);
            updateMissionCard(data.mission);
            break;
        case 'telemetry':
            latestTelemetry.set(data.data.drone_id, data.data);
            // Update UWB ranging lines — only for drones still in droneIdToMission.
            // Guard is essential: backend keeps sending telemetry ~5 s after mission
            // ends. Without it, post-cleanup packets re-create ghost Polyline objects
            // in _uwbLineCollection that have no cleanup path and freeze at the
            // drone's last position permanently.
            if (data.data.gps_denied && data.data.position
                    && droneIdToMission.has(data.data.drone_id)) {
                const gpsd     = data.data.gps_denied;
                const pos      = data.data.position;
                const droneId  = data.data.drone_id;
                const missionId  = droneIdToMission.get(droneId);
                const isSelected = selectedMission === missionId;
                const shouldShow = showUWBRanging || isSelected;

                if (gpsd.active_node_ids && gpsd.active_node_ids.length > 0) {
                    lastRangingData.set(droneId, {
                        lat: pos.latitude, lon: pos.longitude, alt: pos.altitude,
                        nodeIds: gpsd.active_node_ids,
                    });
                    updateRangingLines(droneId, pos, gpsd.active_node_ids, shouldShow);
                } else if (!shouldShow) {
                    _hideAllLinesForDrone(droneId);
                }
            }
            // If this drone is selected, refresh the panel immediately
            if (selectedMission) {
                const m = missions.get(selectedMission);
                if (m && m.droneId === data.data.drone_id) refreshInfoPanel();
            }
            break;
        case 'conflict_resolved':
            updateConflictCount();
            break;
    }
}

// ── Spawn drone on mission ───────────────────────────────────
function spawnMissionDrone(mission, blockedPath) {
    if (!mission.trajectory || !mission.drone_id) return;
    const { waypoints } = mission.trajectory;
    if (!waypoints || waypoints.length < 2) return;

    const missionId = mission.mission_id;
    const droneId   = mission.drone_id;

    // ── Should the safe route start hidden for the reroute animation? ──────
    // If reroute animation will play, hide safe route until animation finishes.
    // Otherwise respect the global toggle.
    const hasBlockedAnim = showRerouteAnimation && blockedPath && blockedPath.length >= 2;
    const initialRouteVisible = hasBlockedAnim ? false : showAllRoutes;

    // ── Reroute animation ─────────────────────────────────────
    // If the direct path was blocked, show it briefly in red before the real route
    if (hasBlockedAnim) {
        const ANIM_ALT = 60;
        const blockedPositions = blockedPath.map(([lat, lon]) =>
            Cesium.Cartesian3.fromDegrees(lon, lat, ANIM_ALT)
        );

        const blockedEntity = viewer.entities.add({
            id: `blocked_${missionId}`,
            polyline: {
                positions: blockedPositions,
                width: 3,
                material: new Cesium.PolylineDashMaterialProperty({
                    color: Cesium.Color.RED.withAlpha(0.9),
                    dashLength: 16.0
                }),
                clampToGround: false
            }
        });

        const midIdx = Math.floor(blockedPath.length / 2);
        const midPt  = blockedPath[midIdx];
        const reroutingLabel = viewer.entities.add({
            id: `rerouting_label_${missionId}`,
            position: Cesium.Cartesian3.fromDegrees(midPt[1], midPt[0], ANIM_ALT + 250),
            label: {
                text: '⚠ REROUTING — NO-FLY ZONE',
                font: 'bold 13px monospace',
                fillColor: Cesium.Color.RED,
                outlineColor: Cesium.Color.BLACK,
                outlineWidth: 3,
                style: Cesium.LabelStyle.FILL_AND_OUTLINE,
                disableDepthTestDistance: Number.POSITIVE_INFINITY,
                showBackground: true,
                backgroundColor: Cesium.Color.fromCssColorString('#1a0000').withAlpha(0.88),
                backgroundPadding: new Cesium.Cartesian2(10, 6)
            }
        });

        // Remove blocked overlay after 2.5 s, reveal real route
        setTimeout(() => {
            viewer.entities.remove(blockedEntity);
            viewer.entities.remove(reroutingLabel);
            const entry = missions.get(missionId);
            if (entry) {
                const visible = showAllRoutes || selectedMission === missionId;
                entry.routeEntity.show = visible;
                entry.waypointMarkers.forEach(wp => { wp.show = visible; });
            }
        }, 2500);
    }

    // Build SampledPositionProperty from waypoint ETAs
    const positionProperty = new Cesium.SampledPositionProperty();
    positionProperty.interpolationAlgorithm = Cesium.LagrangePolynomialApproximation;
    positionProperty.interpolationDegree    = 2;

    let startJulian = null, endJulian = null;
    waypoints.forEach((wp, i) => {
        const t = Cesium.JulianDate.fromDate(new Date(wp.eta * 1000));
        const c = Cesium.Cartesian3.fromDegrees(wp.position.longitude, wp.position.latitude, wp.position.altitude);
        positionProperty.addSample(t, c);
        if (i === 0)                    startJulian = t;
        if (i === waypoints.length - 1) endJulian   = t;
    });

    const availability = new Cesium.TimeIntervalCollection([
        new Cesium.TimeInterval({ start: startJulian, stop: endJulian })
    ]);

    // Drone entity — clicking this triggers the info panel
    const droneEntity = viewer.entities.add({
        id:           `drone_${missionId}`,
        availability,
        position:     positionProperty,
        orientation:  new Cesium.VelocityOrientationProperty(positionProperty),
        billboard: {
            image:    createDroneIcon(),
            scale:    0.55,
            verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
            disableDepthTestDistance: Number.POSITIVE_INFINITY
        },
        label: {
            text: droneId,
            font: '11px monospace',
            fillColor:    Cesium.Color.CYAN,
            outlineColor: Cesium.Color.BLACK,
            outlineWidth: 2,
            style:        Cesium.LabelStyle.FILL_AND_OUTLINE,
            verticalOrigin: Cesium.VerticalOrigin.TOP,
            pixelOffset:  new Cesium.Cartesian2(0, 14),
            disableDepthTestDistance: Number.POSITIVE_INFINITY
        },
        // Short breadcrumb trail
        path: {
            resolution: 1,
            material: new Cesium.PolylineGlowMaterialProperty({ glowPower: 0.2, color: Cesium.Color.CYAN.withAlpha(0.7) }),
            width: 3, leadTime: 0, trailTime: 20
        }
    });

    // Route line — hidden by default, shown on toggle or click
    const routePositions = waypoints.map(wp =>
        Cesium.Cartesian3.fromDegrees(wp.position.longitude, wp.position.latitude, wp.position.altitude)
    );
    const routeEntity = viewer.entities.add({
        id: `route_${missionId}`,
        show: initialRouteVisible,
        polyline: {
            positions: routePositions,
            width: 2,
            material: new Cesium.PolylineGlowMaterialProperty({ glowPower: 0.1, color: Cesium.Color.fromCssColorString('#0088ff').withAlpha(0.6) }),
            clampToGround: false
        }
    });

    // Waypoint dots — also hidden by default
    const waypointMarkers = waypoints.map((wp, i) => viewer.entities.add({
        id: `wp_${missionId}_${i}`,
        show: initialRouteVisible,
        position: Cesium.Cartesian3.fromDegrees(wp.position.longitude, wp.position.latitude, wp.position.altitude),
        point: {
            pixelSize: 5,
            color: Cesium.Color.CYAN.withAlpha(0.8),
            outlineColor: Cesium.Color.WHITE.withAlpha(0.5),
            outlineWidth: 1,
            disableDepthTestDistance: Number.POSITIVE_INFINITY
        }
    }));

    // Pickup & delivery pins
    const firstWp = waypoints[0].position;
    const lastWp  = waypoints[waypoints.length - 1].position;

    const pickupMarker = viewer.entities.add({
        id: `pickup_${missionId}`,
        position: Cesium.Cartesian3.fromDegrees(firstWp.longitude, firstWp.latitude, firstWp.altitude),
        point: { pixelSize: 12, color: Cesium.Color.LIME, outlineColor: Cesium.Color.WHITE, outlineWidth: 2, disableDepthTestDistance: Number.POSITIVE_INFINITY },
        label: { text: 'PICKUP', font: '10px monospace', fillColor: Cesium.Color.LIME, outlineColor: Cesium.Color.BLACK, outlineWidth: 2, style: Cesium.LabelStyle.FILL_AND_OUTLINE, pixelOffset: new Cesium.Cartesian2(0, -20), disableDepthTestDistance: Number.POSITIVE_INFINITY }
    });

    const deliveryMarker = viewer.entities.add({
        id: `delivery_${missionId}`,
        position: Cesium.Cartesian3.fromDegrees(lastWp.longitude, lastWp.latitude, lastWp.altitude),
        point: { pixelSize: 12, color: Cesium.Color.TOMATO, outlineColor: Cesium.Color.WHITE, outlineWidth: 2, disableDepthTestDistance: Number.POSITIVE_INFINITY },
        label: { text: 'DELIVERY', font: '10px monospace', fillColor: Cesium.Color.TOMATO, outlineColor: Cesium.Color.BLACK, outlineWidth: 2, style: Cesium.LabelStyle.FILL_AND_OUTLINE, pixelOffset: new Cesium.Cartesian2(0, -20), disableDepthTestDistance: Number.POSITIVE_INFINITY }
    });

    missions.set(missionId, {
        droneId, droneEntity, routeEntity, waypointMarkers,
        pickupMarker, deliveryMarker, waypoints, positionProperty,
        endEta: waypoints[waypoints.length - 1].eta
    });
    droneIdToMission.set(droneId, missionId);
    updateStats();

    // Auto-cleanup after flight ends
    const msUntilEnd = (waypoints[waypoints.length - 1].eta * 1000) - Date.now();
    setTimeout(() => cleanupMission(missionId), Math.max(0, msUntilEnd) + 5000);
}

// ══════════════════════════════════════════════════════════════
// UWB INFRASTRUCTURE TOWERS
// ══════════════════════════════════════════════════════════════

/** Draw a tower icon onto a canvas and return its dataURL. */
function _createTowerIcon(nodeId, isHighlighted = false) {
    const S = 64;
    const canvas = document.createElement('canvas');
    canvas.width = canvas.height = S;
    const ctx = canvas.getContext('2d');

    const baseColor  = isHighlighted ? '#fb923c' : '#f97316';
    const glowColor  = isHighlighted ? '#fdba74' : '#fb923c';

    // Outer glow ring
    ctx.beginPath();
    ctx.arc(S/2, S/2, 28, 0, Math.PI * 2);
    ctx.strokeStyle = glowColor + '44';
    ctx.lineWidth = 5;
    ctx.stroke();

    // Tower body — triangular mast shape
    ctx.beginPath();
    ctx.moveTo(S/2, 6);           // antenna tip
    ctx.lineTo(S/2 - 14, S - 8); // base left
    ctx.lineTo(S/2 + 14, S - 8); // base right
    ctx.closePath();
    ctx.fillStyle = baseColor + 'cc';
    ctx.fill();
    ctx.strokeStyle = glowColor;
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // Cross-bracing lines on mast
    ctx.strokeStyle = glowColor + '99';
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(S/2 - 5, 18); ctx.lineTo(S/2 + 5, 28); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(S/2 + 5, 18); ctx.lineTo(S/2 - 5, 28); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(S/2 - 9, 32); ctx.lineTo(S/2 + 9, 44); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(S/2 + 9, 32); ctx.lineTo(S/2 - 9, 44); ctx.stroke();

    // Pulsing antenna beacon at tip
    ctx.beginPath();
    ctx.arc(S/2, 6, 4, 0, Math.PI * 2);
    ctx.fillStyle = '#fde68a';
    ctx.fill();

    return canvas.toDataURL();
}

// Pre-render the icon once — reused by all 25 towers
const _TOWER_ICON      = _createTowerIcon();

function initAnchorTowers() {
    const LABEL_COLOR = Cesium.Color.fromCssColorString('#fdba74');

    Object.entries(INFRASTRUCTURE_NODES).forEach(([nodeId, node]) => {

        // ── Ground-clamped billboard icon ─────────────────────
        viewer.entities.add({
            id:       `uwb_tower_${nodeId}`,
            position: Cesium.Cartesian3.fromDegrees(node.lon, node.lat, 0),
            billboard: {
                image:           _TOWER_ICON,
                scale:           0.7,
                verticalOrigin:  Cesium.VerticalOrigin.BOTTOM,
                heightReference: Cesium.HeightReference.RELATIVE_TO_GROUND,
                disableDepthTestDistance: Number.POSITIVE_INFINITY,
                // Visible from high altitude — only fade at extreme zoom-out
                scaleByDistance:        new Cesium.NearFarScalar(200, 1.2, 50000, 0.3),
                translucencyByDistance: new Cesium.NearFarScalar(1000, 1.0, 80000, 0.0),
            },
        });

        // ── Vertical shaft — static two-point polyline ─────────
        const shaftTopAlt = node.alt + 20;
        viewer.entities.add({
            id: `uwb_shaft_${nodeId}`,
            polyline: {
                positions: [
                    Cesium.Cartesian3.fromDegrees(node.lon, node.lat, 2),
                    Cesium.Cartesian3.fromDegrees(node.lon, node.lat, shaftTopAlt),
                ],
                width:    2.5,
                material: new Cesium.PolylineGlowMaterialProperty({
                    glowPower: 0.5,
                    color: Cesium.Color.fromCssColorString('#f97316').withAlpha(0.9),
                }),
                clampToGround: false,
                arcType: Cesium.ArcType.NONE,
            }
        });

        // ── NodeID label ────────────────────────────────────────
        viewer.entities.add({
            id:       `uwb_label_${nodeId}`,
            position: Cesium.Cartesian3.fromDegrees(node.lon, node.lat, 0),
            label: {
                text:            nodeId,
                font:            'bold 11px monospace',
                fillColor:       LABEL_COLOR,
                outlineColor:    Cesium.Color.BLACK,
                outlineWidth:    2,
                style:           Cesium.LabelStyle.FILL_AND_OUTLINE,
                showBackground:  true,
                backgroundColor: Cesium.Color.fromCssColorString('#1a0a00').withAlpha(0.78),
                backgroundPadding: new Cesium.Cartesian2(5, 3),
                pixelOffset:     new Cesium.Cartesian2(0, -52),
                heightReference: Cesium.HeightReference.RELATIVE_TO_GROUND,
                disableDepthTestDistance: Number.POSITIVE_INFINITY,
                scaleByDistance:        new Cesium.NearFarScalar(300, 1.0, 30000, 0.3),
                translucencyByDistance: new Cesium.NearFarScalar(1000, 1.0, 80000, 0.0),
            }
        });
    });

    console.log(`[UWB] ${Object.keys(INFRASTRUCTURE_NODES).length} anchor towers rendered`);
}


// ══════════════════════════════════════════════════════════════
// UWB RANGING LINES
// ══════════════════════════════════════════════════════════════

// ══════════════════════════════════════════════════════════════
// UWB RANGING LINES  (PolylineCollection — scene primitive)
// ══════════════════════════════════════════════════════════════

function _getOrCreateLine(droneId, nodeId, isSelected) {
    if (!_droneNodeLines.has(droneId)) _droneNodeLines.set(droneId, new Map());
    const nodeMap = _droneNodeLines.get(droneId);

    if (!nodeMap.has(nodeId)) {
        const line = _uwbLineCollection.add({
            positions: [Cesium.Cartesian3.ZERO, Cesium.Cartesian3.ZERO],
            width:     isSelected ? 3.5 : 2.5,
            show:      false,
            material:  isSelected ? _UWB_MATERIAL_SEL : _UWB_MATERIAL,
            depthFailMaterial: _UWB_DEPTH_FAIL,
        });
        nodeMap.set(nodeId, line);
    }
    return nodeMap.get(nodeId);
}

function _hideAllLinesForDrone(droneId) {
    const nodeMap = _droneNodeLines.get(droneId);
    if (!nodeMap) return;
    nodeMap.forEach(line => { line.show = false; });
}

function _removeAllLinesForDrone(droneId) {
    const nodeMap = _droneNodeLines.get(droneId);
    if (!nodeMap) return;
    nodeMap.forEach(line => { _uwbLineCollection.remove(line); });
    _droneNodeLines.delete(droneId);
}

function updateRangingLines(droneId, dronePos, nodeIds, visible) {
    const activeSet = new Set(visible ? (nodeIds || []) : []);

    // Hide only lines whose nodeId dropped out of the active set (or visible=false).
    // Never hide-all-then-reshow: with multiple drones at 2 Hz, that causes one
    // drone's lines to be dark for a full render frame when another drone's packet
    // lands in the same JS micro-task turn.
    const nodeMap = _droneNodeLines.get(droneId);
    if (nodeMap) {
        nodeMap.forEach((line, nodeId) => {
            if (!activeSet.has(nodeId)) line.show = false;
        });
    }

    if (!visible || activeSet.size === 0) return;

    const missionId  = droneIdToMission.get(droneId);
    const isSelected = selectedMission === missionId;

    const dLon = dronePos.longitude ?? dronePos.lon;
    const dLat = dronePos.latitude  ?? dronePos.lat;
    const dAlt = dronePos.altitude  ?? dronePos.alt ?? 50;
    const dronePosC3 = Cesium.Cartesian3.fromDegrees(dLon, dLat, dAlt);

    activeSet.forEach(nodeId => {
        const node = INFRASTRUCTURE_NODES[nodeId];
        if (!node) return;

        const nodeTipC3 = Cesium.Cartesian3.fromDegrees(
            node.lon, node.lat, node.alt + 20
        );

        const line     = _getOrCreateLine(droneId, nodeId, isSelected);
        line.positions = [dronePosC3, nodeTipC3];   // update live position every tick
        line.show      = true;
        line.width     = isSelected ? 3.5 : 2.5;
        line.material  = isSelected ? _UWB_MATERIAL_SEL : _UWB_MATERIAL;
    });
}

function _redrawAllRangingLines() {
    lastRangingData.forEach((data, droneId) => {
        // Skip drones that have already been cleaned up — their primitives are gone.
        // Also prune stale entries so they don't accumulate across long sessions.
        if (!droneIdToMission.has(droneId)) {
            lastRangingData.delete(droneId);
            return;
        }
        const missionId  = droneIdToMission.get(droneId);
        const isSelected = selectedMission === missionId;
        const shouldShow = showUWBRanging || isSelected;
        updateRangingLines(
            droneId,
            { lat: data.lat, lon: data.lon, alt: data.alt },
            data.nodeIds,
            shouldShow
        );
    });
}

/**
 * Launch both legs of a conflict demo pair with a short gap between them.
 * The gap (200 ms) is intentionally small so both 4D trajectories are
 * registered before the second one's CPA check runs — guaranteeing overlap.
 */
async function loadConflictScenario(id) {
    const s = DEMO_SCENARIOS[id];
    if (!s || !s.pair) return;

    for (const leg of s.pair) {
        await fetch(`${API_URL}/api/delivery/request`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                pickup:   { latitude: leg.pickup.lat,   longitude: leg.pickup.lon,   altitude: MIN_ALTITUDE },
                delivery: { latitude: leg.delivery.lat, longitude: leg.delivery.lon, altitude: MIN_ALTITUDE },
            })
        }).catch(e => console.error(`Conflict scenario ${id} leg failed:`, e));

        // Short gap — enough for the first mission to be stored in flight_plans_4d
        // before the second request's CPA check runs against it.
        await new Promise(r => setTimeout(r, 200));
    }
}

async function loadAllConflictScenarios() {
    for (const id of [12, 13, 14, 15, 16]) {
        await loadConflictScenario(id);
        await new Promise(r => setTimeout(r, 1000));  // gap between pairs
    }
}


// ══════════════════════════════════════════════════════════════
// UWB TOGGLE
// ══════════════════════════════════════════════════════════════

function toggleUWBRanging() {
    showUWBRanging = !showUWBRanging;
    const btn = document.getElementById('uwbToggleBtn');
    btn.classList.toggle('uwb-on', showUWBRanging);
    btn.querySelector('.toggle-label').textContent =
        showUWBRanging ? 'UWB Ranging: ON' : 'UWB Ranging: OFF';
    _redrawAllRangingLines();
}


function cleanupMission(missionId) {
    const m = missions.get(missionId);
    if (!m) return;

    [m.droneEntity, m.routeEntity, m.pickupMarker, m.deliveryMarker, ...m.waypointMarkers]
        .forEach(e => viewer.entities.remove(e));

    // Clean up UWB ranging lines for this drone
    _removeAllLinesForDrone(m.droneId);
    lastRangingData.delete(m.droneId);

    droneIdToMission.delete(m.droneId);
    missions.delete(missionId);

    if (selectedMission === missionId) deselectDrone();
    updateStats();
    console.log(`[${missionId}] Cleaned up`);
}

// ── Toggle: show/hide all routes ─────────────────────────────
function toggleAllRoutes() {
    showAllRoutes = !showAllRoutes;

    const btn = document.getElementById('routeToggleBtn');
    btn.classList.toggle('toggle-on', showAllRoutes);
    btn.querySelector('.toggle-label').textContent = showAllRoutes ? 'All Routes: ON' : 'All Routes: OFF';

    missions.forEach((m) => {
        m.routeEntity.show = showAllRoutes;
        m.waypointMarkers.forEach(wp => { wp.show = showAllRoutes; });
    });

    // If turning off and a drone is selected, keep only its route visible
    if (!showAllRoutes && selectedMission) {
        const sel = missions.get(selectedMission);
        if (sel) {
            sel.routeEntity.show = true;
            sel.waypointMarkers.forEach(wp => { wp.show = true; });
        }
    }
}

function toggleRerouteAnimation() {
    showRerouteAnimation = !showRerouteAnimation;
    const btn = document.getElementById('rerouteToggleBtn');
    btn.classList.toggle('toggle-on', showRerouteAnimation);
    btn.querySelector('.toggle-label').textContent = showRerouteAnimation
        ? 'Reroute Anim: ON'
        : 'Reroute Anim: OFF';
}

// ── Click to select a drone ───────────────────────────────────
viewer.selectedEntityChanged.addEventListener((entity) => {
    if (!entity) {
        deselectDrone();
        return;
    }

    // Match entity id like "drone_mission_XXXXXXXX"
    const idStr = entity.id || '';
    const match = idStr.match(/^drone_(mission_.+)$/);
    if (!match) {
        deselectDrone();
        return;
    }

    const missionId = match[1];
    selectDrone(missionId);
});

function selectDrone(missionId) {
    // Deselect previous
    if (selectedMission && selectedMission !== missionId) {
        const prev = missions.get(selectedMission);
        if (prev && !showAllRoutes) {
            prev.routeEntity.show = false;
            prev.waypointMarkers.forEach(wp => { wp.show = false; });
        }
        // Reset icon
        if (prev) prev.droneEntity.billboard.image = createDroneIcon(false);
        // Hide ranging lines for the previously selected drone (unless global toggle is on)
        if (!showUWBRanging) {
            const prevMission = missions.get(selectedMission);
            if (prevMission) _hideAllLinesForDrone(prevMission.droneId);
        }
    }

    selectedMission = missionId;
    const m = missions.get(missionId);
    if (!m) return;

    // Always show this drone's route when selected
    m.routeEntity.show = true;
    m.waypointMarkers.forEach(wp => { wp.show = true; });

    // Always show this drone's ranging lines when selected
    const cached = lastRangingData.get(m.droneId);
    if (cached) {
        updateRangingLines(m.droneId,
            { lat: cached.lat, lon: cached.lon, alt: cached.alt },
            cached.nodeIds, true);
    }

    // Highlight icon
    m.droneEntity.billboard.image = createDroneIcon(true);

    // Show info panel
    document.getElementById('droneInfoPanel').style.display = 'block';
    document.getElementById('droneInfoTitle').textContent   = m.droneId;
    refreshInfoPanel();

    // Start live refresh
    clearInterval(infoUpdateTimer);
    infoUpdateTimer = setInterval(refreshInfoPanel, 1000);
}

function deselectDrone() {
    if (selectedMission) {
        const m = missions.get(selectedMission);
        if (m && !showAllRoutes) {
            m.routeEntity.show = false;
            m.waypointMarkers.forEach(wp => { wp.show = false; });
        }
        if (m) {
            m.droneEntity.billboard.image = createDroneIcon(false);
            // Hide ranging lines unless global toggle keeps them visible
            if (!showUWBRanging) _hideAllLinesForDrone(m.droneId);
        }
    }
    selectedMission = null;
    clearInterval(infoUpdateTimer);
    document.getElementById('droneInfoPanel').style.display = 'none';
    // Clear Cesium's selection highlight too
    viewer.selectedEntity = undefined;
}

function refreshInfoPanel() {
    if (!selectedMission) return;
    const m = missions.get(selectedMission);
    if (!m) return;

    const tel = latestTelemetry.get(m.droneId);

    let altitude = '—', speed = '—', battery = '—', lat = '—', lon = '—';

    // ── Position & Altitude from SampledPositionProperty ──
    const now  = Cesium.JulianDate.now();
    const pos3d = m.positionProperty.getValue(now);

    if (pos3d) {
        const carto = Cesium.Cartographic.fromCartesian(pos3d);
        lat      = Cesium.Math.toDegrees(carto.latitude).toFixed(5);
        lon      = Cesium.Math.toDegrees(carto.longitude).toFixed(5);
        altitude = `${Math.round(carto.height)} m`;

        const tMinus1 = Cesium.JulianDate.addSeconds(now, -1, new Cesium.JulianDate());
        const posPrev = m.positionProperty.getValue(tMinus1);
        if (posPrev) {
            const cartoPrev = Cesium.Cartographic.fromCartesian(posPrev);
            const R    = 6371000;
            const dLat = carto.latitude  - cartoPrev.latitude;
            const dLon = carto.longitude - cartoPrev.longitude;
            const a    = Math.sin(dLat/2)**2 +
                         Math.cos(cartoPrev.latitude) * Math.cos(carto.latitude) * Math.sin(dLon/2)**2;
            const hDist = R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
            const vDist = Math.abs(carto.height - cartoPrev.height);
            speed = `${Math.sqrt(hDist**2 + vDist**2).toFixed(1)} m/s`;
        }
    }

    if (tel) battery = `${Math.round(tel.battery_level)}%`;

    const secLeft = Math.max(0, m.endEta - Date.now() / 1000);
    const etaStr  = secLeft > 0
        ? `${Math.floor(secLeft / 60)}m ${Math.floor(secLeft % 60)}s`
        : 'Arrived';

    document.getElementById('infoLat').textContent      = lat;
    document.getElementById('infoLon').textContent      = lon;
    document.getElementById('infoAltitude').textContent = altitude;
    document.getElementById('infoSpeed').textContent    = speed;
    document.getElementById('infoBattery').textContent  = battery;
    document.getElementById('infoETA').textContent      = etaStr;
    document.getElementById('infoMission').textContent  = selectedMission;
    document.getElementById('infoWaypoints').textContent = `${m.waypoints.length} waypoints`;

    // ── GPS-Denied / UWB fields ──────────────────────────────────────────────
    try {
        const gpsd = tel ? tel.gps_denied : null;
        if (gpsd) {
            const nodeCount = gpsd.anchor_nodes_used;
            document.getElementById('infoAnchorCount').textContent =
                (nodeCount != null && nodeCount > 0) ? nodeCount : '—';

            const hdopOk  = gpsd.hdop_ok;
            const hdopVal = hdopOk === null || hdopOk === undefined ? null
                          : (hdopOk ? '✓ OK' : '✗ DR');
            _updateDOPBar('infoHDOPBar', 'infoHDOPVal', hdopOk, hdopVal);

            const vdopOk  = gpsd.vdop_ok;
            const vdopVal = vdopOk === null || vdopOk === undefined ? null
                          : (vdopOk ? '✓ OK' : '✗ BARO');
            _updateDOPBar('infoVDOPBar', 'infoVDOPVal', vdopOk, vdopVal);

            const corrEl = document.getElementById('infoCorrection');
            if (corrEl && gpsd.correction_m != null) {
                corrEl.innerHTML = `<span class="correction-tag">${Number(gpsd.correction_m).toFixed(2)} m</span>`;
            }

            const rmsEl = document.getElementById('infoRMS');
            if (rmsEl && gpsd.residual_rms != null) {
                const rms = Number(gpsd.residual_rms).toFixed(3);
                rmsEl.textContent = `${rms} m`;
                rmsEl.style.color = gpsd.residual_rms < 2.0 ? '#10b981'
                                  : gpsd.residual_rms < 5.0 ? '#f59e0b'
                                  : '#ef4444';
            }
        }
    } catch (e) {
        console.warn('[UWB panel] refreshInfoPanel GPS section error:', e);
    }
}

/**
 * Update a DOP progress bar and label element.
 * okFlag: true = geometry OK (green), false = fallback (red), null = unknown (grey)
 */
function _updateDOPBar(barId, valId, okFlag, label) {
    const bar = document.getElementById(barId);
    const val = document.getElementById(valId);
    if (!bar || !val) return;

    if (okFlag === null || okFlag === undefined) {
        bar.style.width = '0%';
        bar.className = 'dop-bar-fill';
        val.textContent = '—';
        val.style.color = 'var(--text-tertiary)';
        return;
    }
    if (okFlag) {
        bar.style.width = '60%';
        bar.className = 'dop-bar-fill ok';
        val.textContent = label || '✓ OK';
        val.style.color = '#10b981';
    } else {
        bar.style.width = '100%';
        bar.className = 'dop-bar-fill bad';
        val.textContent = label || '✗ POOR';
        val.style.color = '#ef4444';
    }
}

// ── Drone icon ───────────────────────────────────────────────
function createDroneIcon(selected = false) {
    const canvas = document.createElement('canvas');
    canvas.width = canvas.height = 64;
    const ctx   = canvas.getContext('2d');
    const color = selected ? '#ffffff' : '#00d9ff';
    const glow  = selected ? '#ffd700' : '#00d9ff';

    ctx.beginPath();
    ctx.arc(32, 32, 28, 0, Math.PI * 2);
    ctx.strokeStyle = glow + '44';
    ctx.lineWidth = 6;
    ctx.stroke();

    ctx.beginPath();
    ctx.arc(32, 32, 18, 0, Math.PI * 2);
    ctx.fillStyle = selected ? '#ffd700' : color;
    ctx.fill();

    ctx.beginPath();
    ctx.arc(32, 32, 6, 0, Math.PI * 2);
    ctx.fillStyle = '#ffffff';
    ctx.fill();

    ctx.strokeStyle = selected ? '#ffd700' : color;
    ctx.lineWidth = 3;
    [[-1,-1],[1,-1],[-1,1],[1,1]].forEach(([dx, dy]) => {
        ctx.beginPath();
        ctx.moveTo(32 + dx*14, 32 + dy*14);
        ctx.lineTo(32 + dx*26, 32 + dy*26);
        ctx.stroke();
        ctx.beginPath();
        ctx.arc(32 + dx*28, 32 + dy*28, 4, 0, Math.PI * 2);
        ctx.fillStyle = selected ? '#ffd700' : color;
        ctx.fill();
    });

    return canvas.toDataURL();
}

// ── Geofencing ───────────────────────────────────────────────
function loadGeofencing(geofencing) {
    geofencing.no_fly_zones.forEach(zone => {
        const positions = zone.polygon.map(([lat, lon]) => Cesium.Cartesian3.fromDegrees(lon, lat));
        viewer.entities.add({
            polygon: {
                hierarchy: new Cesium.PolygonHierarchy(positions),
                material: Cesium.Color.RED.withAlpha(0.25),
                outline: true, outlineColor: Cesium.Color.RED, outlineWidth: 2,
                height: 0, extrudedHeight: 150
            }
        });
        const cLat = zone.polygon.reduce((s,p) => s + p[0], 0) / zone.polygon.length;
        const cLon = zone.polygon.reduce((s,p) => s + p[1], 0) / zone.polygon.length;
        viewer.entities.add({
            position: Cesium.Cartesian3.fromDegrees(cLon, cLat, 160),
            label: { text: `🚫 ${zone.name}`, font: '13px monospace', fillColor: Cesium.Color.RED, outlineColor: Cesium.Color.BLACK, outlineWidth: 2, style: Cesium.LabelStyle.FILL_AND_OUTLINE, disableDepthTestDistance: Number.POSITIVE_INFINITY }
        });
    });

    geofencing.sensitive_areas.forEach(area => {
        const positions = area.polygon.map(([lat, lon]) => Cesium.Cartesian3.fromDegrees(lon, lat));
        viewer.entities.add({
            polygon: {
                hierarchy: new Cesium.PolygonHierarchy(positions),
                material: Cesium.Color.ORANGE.withAlpha(0.2),
                outline: true, outlineColor: Cesium.Color.ORANGE, outlineWidth: 2,
                height: 0, extrudedHeight: 100
            }
        });
    });
}

// ── Mission list card ────────────────────────────────────────
function updateMissionCard(mission) {
    const list = document.getElementById('missionList');
    const placeholder = list.querySelector('[data-placeholder]');
    if (placeholder) placeholder.remove();

    const existing = document.getElementById(`mc_${mission.mission_id}`);
    if (existing) existing.remove();

    const card = document.createElement('div');
    card.className = 'mission-card';
    card.id = `mc_${mission.mission_id}`;
    // Clicking the card also selects the drone
    card.style.cursor = 'pointer';
    card.onclick = () => selectDrone(mission.mission_id);
    card.innerHTML = `
        <div class="mission-header">
            <span class="mission-id">${mission.mission_id}</span>
            <span class="status-badge">${mission.status.replace(/_/g,' ').toUpperCase()}</span>
        </div>
        <div class="mission-info">
            Drone: ${mission.drone_id || 'Unassigned'}<br>
            Distance: ${mission.trajectory ? (mission.trajectory.total_distance/1000).toFixed(2) : 'N/A'} km<br>
            ETA: ${mission.trajectory ? Math.floor(mission.trajectory.total_time/60) : 'N/A'} min
        </div>
    `;
    list.appendChild(card);
}

// ── Stats ────────────────────────────────────────────────────
function updateStats() {
    document.getElementById('activeDroneCount').textContent = missions.size;
}

function updateConflictCount() {
    fetch(`${API_URL}/api/system/status`)
        .then(r => r.json())
        .then(d => {
            document.getElementById('missionCount').textContent      = d.active_missions;
            document.getElementById('conflictCount').textContent     = d.conflicts_detected;
            document.getElementById('totalFlights').textContent      = d.total_flights_today;
            document.getElementById('resolvedConflicts').textContent = d.conflicts_resolved;
        }).catch(() => {});
}

// ── Demo / manual controls ───────────────────────────────────
function loadDemoScenario(n) {
    const s = DEMO_SCENARIOS[n];
    document.getElementById('pickupLat').value   = s.pickup.lat;
    document.getElementById('pickupLon').value   = s.pickup.lon;
    document.getElementById('deliveryLat').value = s.delivery.lat;
    document.getElementById('deliveryLon').value = s.delivery.lon;
    createDelivery();
}

async function loadAllDemoScenarios() {
    for (let i = 1; i <= 5; i++) {
        const s = DEMO_SCENARIOS[i];
        await fetch(`${API_URL}/api/delivery/request`, {
            method: 'POST', headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                pickup:   { latitude: s.pickup.lat,   longitude: s.pickup.lon,   altitude: MIN_ALTITUDE },
                delivery: { latitude: s.delivery.lat, longitude: s.delivery.lon, altitude: MIN_ALTITUDE }
            })
        }).catch(e => console.error(`Scenario ${i} failed:`, e));
        await new Promise(r => setTimeout(r, 600));
    }
}

async function loadAllRerouteScenarios() {
    for (let i = 6; i <= 11; i++) {
        const s = DEMO_SCENARIOS[i];
        await fetch(`${API_URL}/api/delivery/request`, {
            method: 'POST', headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                pickup:   { latitude: s.pickup.lat,   longitude: s.pickup.lon,   altitude: MIN_ALTITUDE },
                delivery: { latitude: s.delivery.lat, longitude: s.delivery.lon, altitude: MIN_ALTITUDE }
            })
        }).catch(e => console.error(`Scenario ${i} failed:`, e));
        await new Promise(r => setTimeout(r, 2000));  // enough gap for reroute animation
    }
}

async function createDelivery() {
    const pLat = parseFloat(document.getElementById('pickupLat').value);
    const pLon = parseFloat(document.getElementById('pickupLon').value);
    const dLat = parseFloat(document.getElementById('deliveryLat').value);
    const dLon = parseFloat(document.getElementById('deliveryLon').value);

    const pWater = isOverWater(pLat, pLon);
    const dWater = isOverWater(dLat, dLon);
    if (pWater) { showError(`Pickup is over ${pWater} — choose a point on land`); return; }
    if (dWater) { showError(`Delivery is over ${dWater} — choose a point on land`); return; }

    const request = {
        pickup:   { latitude: parseFloat(document.getElementById('pickupLat').value),   longitude: parseFloat(document.getElementById('pickupLon').value),   altitude: MIN_ALTITUDE },
        delivery: { latitude: parseFloat(document.getElementById('deliveryLat').value), longitude: parseFloat(document.getElementById('deliveryLon').value), altitude: MIN_ALTITUDE }
    };
    try {
        const res = await fetch(`${API_URL}/api/delivery/request`, {
            method: 'POST', headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(request)
        });
        if (!res.ok) {
            const body = await res.text();
            let msg = body;
            try { msg = JSON.parse(body).detail || body; } catch(e) {}
            showError(msg);
        }
    } catch (e) {
        alert(`Network error: ${e.message}`);
    }
}

// ── Client-side water body check ─────────────────────────────
// Mirrors the server-side is_over_water() in geofencing.py.
// Gives instant feedback before the form is submitted.
const _BAY_BOUNDARY = [
    [37.50,-122.315],[37.56,-122.325],[37.58,-122.330],[37.60,-122.335],
    [37.62,-122.345],[37.65,-122.358],[37.68,-122.365],[37.71,-122.370],
    [37.74,-122.376],[37.77,-122.383],[37.79,-122.390],[37.81,-122.397],
    [37.83,-122.408],[37.90,-122.430],
];
const _PACIFIC_LON = -122.511;

function _bayThreshold(lat) {
    const pts = _BAY_BOUNDARY;
    if (lat <= pts[0][0])              return pts[0][1];
    if (lat >= pts[pts.length-1][0])   return pts[pts.length-1][1];
    for (let i = 0; i < pts.length-1; i++) {
        const [la0,lo0] = pts[i], [la1,lo1] = pts[i+1];
        if (la0 <= lat && lat <= la1) {
            const t = (lat-la0)/(la1-la0);
            return lo0 + t*(lo1-lo0);
        }
    }
    return pts[pts.length-1][1];
}

function isOverWater(lat, lon) {
    if (lon <= _PACIFIC_LON)           return 'the Pacific Ocean';
    if (lon > _bayThreshold(lat))      return 'San Francisco Bay';
    return null;
}

function validateCoords() {
    const pLat = parseFloat(document.getElementById('pickupLat').value);
    const pLon = parseFloat(document.getElementById('pickupLon').value);
    const dLat = parseFloat(document.getElementById('deliveryLat').value);
    const dLon = parseFloat(document.getElementById('deliveryLon').value);

    const pWarn  = document.getElementById('pickupWaterWarn');
    const dWarn  = document.getElementById('deliveryWaterWarn');
    const btn    = document.getElementById('deliverySubmitBtn');

    const pWater = (!isNaN(pLat) && !isNaN(pLon)) ? isOverWater(pLat, pLon) : null;
    const dWater = (!isNaN(dLat) && !isNaN(dLon)) ? isOverWater(dLat, dLon) : null;

    pWarn.textContent = pWater ? `⚠ Pickup is over ${pWater}` : '';
    dWarn.textContent = dWater ? `⚠ Delivery is over ${dWater}` : '';

    const blocked = pWater || dWater;
    btn.disabled = !!blocked;
    btn.style.opacity = blocked ? '0.45' : '';
    btn.style.cursor  = blocked ? 'not-allowed' : '';
}

// ── Error toast ───────────────────────────────────────────────
function showError(msg) {
    const existing = document.getElementById('errorToast');
    if (existing) existing.remove();

    const toast = document.createElement('div');
    toast.id = 'errorToast';
    toast.className = 'error-toast';
    toast.innerHTML = `<span class="error-toast-icon">⚠</span><span class="error-toast-msg">${msg}</span>`;
    document.body.appendChild(toast);

    // Animate in
    requestAnimationFrame(() => toast.classList.add('error-toast-show'));
    // Auto-dismiss after 4s
    setTimeout(() => {
        toast.classList.remove('error-toast-show');
        setTimeout(() => toast.remove(), 300);
    }, 4000);
}

// ── Boot ─────────────────────────────────────────────────────
// initAnchorTowers() is called inside the 'initial_state' WS handler,
// after INFRASTRUCTURE_NODES has been populated from the backend.
connectWebSocket();
setInterval(updateConflictCount, 5000);
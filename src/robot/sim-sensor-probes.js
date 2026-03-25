/**
 * Simulation sensor probe positions for the Robbo 2D simulator.
 *
 * Coordinate contract (schema version 2+):
 * - localX / localY are offsets from the sprite rotation center in Scratch stage units,
 *   expressed as if the sprite had size === 100 (100% of native costume).
 * - World-space offset = rotate(localX, localY) by the sprite heading, then multiply by (size / 100),
 *   matching how Scratch scales costumes via RenderedTarget.size.
 *
 * Legacy storage (raw JSON array): values were stage offsets independent of sprite size,
 * tuned for a ~25% library robot. On load they are converted to the contract above by
 * multiplying coordinates by (100 / LEGACY_CALIBRATION_SIZE_PERCENT).
 *
 * v2 payload may also include touchMaxHitDistance: max ray distance (stage units, integer)
 * at which the simulated touch sensor still reads 100 (same step as getDistToWall).
 */

const STORAGE_KEY = 'rs3.simSensorProbes';
const SCHEMA_VERSION = 2;

/** RobboPlatform in sprites.json uses scale 0.25; legacy probe numbers matched that visual size. */
const LEGACY_CALIBRATION_SIZE_PERCENT = 25;

/**
 * Default probes in v2 semantics (100% basis). Equivalent to former defaults at size 25% after × (100/25).
 * @type {Array<{localX: number, localY: number, direction: 'forward'|'backward'}>}
 */
const DEFAULT_SIM_SENSOR_PROBES = [
    {localX: 4, localY: 152, direction: 'forward'},
    {localX: 72, localY: 120, direction: 'forward'},
    {localX: 72, localY: -152, direction: 'backward'},
    {localX: -56, localY: -152, direction: 'backward'},
    {localX: -66, localY: 120, direction: 'forward'}
];

/** Default max hit distance along touch ray (stage units) for touch sensor value 100. */
const DEFAULT_SIM_TOUCH_MAX_HIT_DISTANCE = 10;

/** Upper bound for touchMaxHitDistance (matches typical getDistToWall max scan). */
const MAX_SIM_TOUCH_MAX_HIT_DISTANCE = 100;

/** Stage units for debug ray at size 100%; keeps apparent arrow length ~10 at size 25%. */
const SIM_SENSOR_DEBUG_RAY_LENGTH_AT_SIZE100 = 40;

/**
 * @param {*} probe
 * @param {{localX: number, localY: number, direction: string}} fallbackProbe
 */
function sanitizeSimSensorProbe (probe, fallbackProbe) {
    const localX = Number(probe && probe.localX);
    const localY = Number(probe && probe.localY);
    const direction = (probe && probe.direction === 'backward') ? 'backward' : 'forward';
    return {
        localX: Number.isFinite(localX) ? localX : fallbackProbe.localX,
        localY: Number.isFinite(localY) ? localY : fallbackProbe.localY,
        direction: direction
    };
}

/**
 * @param {*} value
 * @returns {number} integer in [0, MAX_SIM_TOUCH_MAX_HIT_DISTANCE]
 */
function sanitizeTouchMaxHitDistance (value) {
    const n = Math.round(Number(value));
    if (!Number.isFinite(n)) {
        return DEFAULT_SIM_TOUCH_MAX_HIT_DISTANCE;
    }
    if (n < 0) {
        return 0;
    }
    if (n > MAX_SIM_TOUCH_MAX_HIT_DISTANCE) {
        return MAX_SIM_TOUCH_MAX_HIT_DISTANCE;
    }
    return n;
}

function cloneDefaultProbes () {
    return DEFAULT_SIM_SENSOR_PROBES.map(p => ({
        localX: p.localX,
        localY: p.localY,
        direction: p.direction
    }));
}

/**
 * Stage-space offset from rotation center for a probe, given sprite size percent.
 */
function stageOffsetFromProbeAt100Percent (probe, sizePercent, forwardX, forwardY, rightX, rightY) {
    const scale = (Number.isFinite(sizePercent) && sizePercent > 0) ? sizePercent / 100 : 1;
    const dx = (rightX * probe.localX + forwardX * probe.localY) * scale;
    const dy = (rightY * probe.localX + forwardY * probe.localY) * scale;
    return {dx, dy};
}

function simSensorDebugRayLengthStageUnits (sizePercent) {
    const s = Number.isFinite(sizePercent) ? sizePercent : 100;
    return (SIM_SENSOR_DEBUG_RAY_LENGTH_AT_SIZE100 * s) / 100;
}

function migrateLegacyProbeList (rawList) {
    const defaults = cloneDefaultProbes();
    const factor = 100 / LEGACY_CALIBRATION_SIZE_PERCENT;
    if (!Array.isArray(rawList)) {
        return defaults;
    }
    return defaults.map((def, idx) => {
        const raw = rawList[idx];
        if (!raw || typeof raw !== 'object') {
            return def;
        }
        const lx = Number(raw.localX);
        const ly = Number(raw.localY);
        const migrated = {
            localX: Number.isFinite(lx) ? lx * factor : def.localX,
            localY: Number.isFinite(ly) ? ly * factor : def.localY,
            direction: raw.direction
        };
        return sanitizeSimSensorProbe(migrated, def);
    });
}

function normalizeV2Probes (rawProbes) {
    const defaults = cloneDefaultProbes();
    if (!Array.isArray(rawProbes)) {
        return defaults;
    }
    return defaults.map((def, idx) => sanitizeSimSensorProbe(rawProbes[idx], def));
}

/**
 * @param {Array} probes
 * @param {number} touchMaxHitDistance
 */
function saveSimSensorCalibration (probes, touchMaxHitDistance) {
    try {
        const storage = (typeof globalThis !== 'undefined') ? globalThis.localStorage : null;
        if (!storage || !Array.isArray(probes)) {
            return;
        }
        const touch = sanitizeTouchMaxHitDistance(touchMaxHitDistance);
        const payload = {
            schemaVersion: SCHEMA_VERSION,
            probes: probes.map(p => ({
                localX: p.localX,
                localY: p.localY,
                direction: p.direction
            })),
            touchMaxHitDistance: touch
        };
        storage.setItem(STORAGE_KEY, JSON.stringify(payload));
    } catch (e) {
        // Ignore storage write errors in restricted environments.
    }
}

/**
 * Saves probes; if touchMaxHitDistance omitted, keeps existing value from storage (does not default to 1).
 * Prefer saveSimSensorCalibration from app code when both values are known.
 */
function saveSimSensorProbes (probes, touchMaxHitDistance) {
    let touch = touchMaxHitDistance;
    if (touch === undefined || touch === null) {
        try {
            const storage = (typeof globalThis !== 'undefined') ? globalThis.localStorage : null;
            if (storage) {
                const raw = storage.getItem(STORAGE_KEY);
                if (raw) {
                    const parsed = JSON.parse(raw);
                    if (parsed && (parsed.touchMaxHitDistance !== undefined && parsed.touchMaxHitDistance !== null)) {
                        touch = sanitizeTouchMaxHitDistance(parsed.touchMaxHitDistance);
                    }
                }
            }
        } catch (e) {
            // fall through to default
        }
        if (touch === undefined || touch === null) {
            touch = DEFAULT_SIM_TOUCH_MAX_HIT_DISTANCE;
        }
    }
    saveSimSensorCalibration(probes, touch);
}

/**
 * @returns {{ probes: Array, touchMaxHitDistance: number }}
 */
function loadSimSensorCalibration () {
    const defaults = cloneDefaultProbes();
    const defaultTouch = DEFAULT_SIM_TOUCH_MAX_HIT_DISTANCE;
    try {
        const storage = (typeof globalThis !== 'undefined') ? globalThis.localStorage : null;
        if (!storage) {
            return {probes: defaults, touchMaxHitDistance: defaultTouch};
        }
        const raw = storage.getItem(STORAGE_KEY);
        if (!raw) {
            return {probes: defaults, touchMaxHitDistance: defaultTouch};
        }
        const parsed = JSON.parse(raw);
        let probes;
        let touchMaxHitDistance = defaultTouch;
        let needsResave = false;

        if (Array.isArray(parsed)) {
            probes = migrateLegacyProbeList(parsed);
            needsResave = true;
        } else if (parsed && typeof parsed === 'object') {
            const ver = Number(parsed.schemaVersion);
            if (ver >= SCHEMA_VERSION && Array.isArray(parsed.probes)) {
                probes = normalizeV2Probes(parsed.probes);
                if (parsed.touchMaxHitDistance !== undefined && parsed.touchMaxHitDistance !== null) {
                    touchMaxHitDistance = sanitizeTouchMaxHitDistance(parsed.touchMaxHitDistance);
                } else {
                    needsResave = true;
                }
            } else if (Array.isArray(parsed.probes)) {
                probes = migrateLegacyProbeList(parsed.probes);
                needsResave = true;
            } else {
                return {probes: defaults, touchMaxHitDistance: defaultTouch};
            }
        } else {
            return {probes: defaults, touchMaxHitDistance: defaultTouch};
        }

        if (needsResave) {
            saveSimSensorCalibration(probes, touchMaxHitDistance);
        }
        return {probes, touchMaxHitDistance};
    } catch (e) {
        return {probes: defaults, touchMaxHitDistance: defaultTouch};
    }
}

function loadSimSensorProbes () {
    return loadSimSensorCalibration().probes;
}

module.exports = {
    STORAGE_KEY,
    SCHEMA_VERSION,
    DEFAULT_SIM_SENSOR_PROBES,
    DEFAULT_SIM_TOUCH_MAX_HIT_DISTANCE,
    MAX_SIM_TOUCH_MAX_HIT_DISTANCE,
    LEGACY_CALIBRATION_SIZE_PERCENT,
    sanitizeSimSensorProbe,
    sanitizeTouchMaxHitDistance,
    cloneDefaultProbes,
    loadSimSensorCalibration,
    saveSimSensorCalibration,
    loadSimSensorProbes,
    saveSimSensorProbes,
    stageOffsetFromProbeAt100Percent,
    simSensorDebugRayLengthStageUnits
};

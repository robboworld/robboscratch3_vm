/**
 * SB3 project.json meta.robboSimulator (v1) — save/load helpers.
 * @module robbo/robbo-simulator-project-meta
 */

const log = require('../util/log');

const META_VERSION = 1;

/** Robot palette sensor ids accepted by RCA (aligned with SensorChooseWindowComponent). */
const ALLOWED_ROBOT_SENSOR_NAMES = new Set([
    'nosensor',
    'line',
    'led',
    'light',
    'touch',
    'proximity',
    'ultrasonic',
    'color'
]);

const SLOT_COUNT = 5;

/**
 * @param {string} name
 * @returns {boolean}
 */
const isAllowedSensorName = name => typeof name === 'string' && ALLOWED_ROBOT_SENSOR_NAMES.has(name.toLowerCase());

/**
 * @param {*} raw
 * @returns {{sensor_name: string, sensor_active: boolean, is_sensor_version_new: boolean}}
 */
const normalizeSavedSlot = raw => {
    let sensorName = 'nosensor';
    if (raw && typeof raw.sensor_name === 'string') {
        const n = raw.sensor_name.toLowerCase();
        if (isAllowedSensorName(n)) {
            sensorName = n;
        } else {
            log.warn(`robboSimulator: invalid sensor_name "${raw.sensor_name}", using nosensor`);
        }
    }
    const sensorActive = !!(raw && raw.sensor_active);
    const isNew = !!(raw && raw.is_sensor_version_new);
    if (sensorName === 'nosensor' && sensorActive) {
        return {sensor_name: 'nosensor', sensor_active: false, is_sensor_version_new: isNew};
    }
    return {sensor_name: sensorName, sensor_active: sensorActive, is_sensor_version_new: isNew};
};

/**
 * Build v1 meta block for project.json from GUI/runtime snapshot.
 * @param {object} opts
 * @param {boolean} opts.simEnabled
 * @param {boolean} opts.extensionPackActivated
 * @param {Array<object>} opts.sensors - five slots from Redux robot_sensors
 * @returns {object}
 */
const buildRobboSimulatorMetaForSave = opts => {
    const simEnabled = !!opts.simEnabled;
    const extensionPackActivated = !!opts.extensionPackActivated;
    const defaultSlot = () => ({
        sensor_name: 'nosensor',
        sensor_active: false,
        is_sensor_version_new: false
    });
    let slots = Array.isArray(opts.sensors) ? opts.sensors.slice(0, SLOT_COUNT) : [];
    while (slots.length < SLOT_COUNT) {
        slots.push(defaultSlot());
    }
    const sensors = [];
    for (let i = 0; i < SLOT_COUNT; i++) {
        const s = slots[i] || defaultSlot();
        sensors.push({
            sensor_name: typeof s.sensor_name === 'string' ? s.sensor_name.toLowerCase() : 'nosensor',
            sensor_active: !!s.sensor_active,
            is_sensor_version_new: !!s.is_sensor_version_new
        });
    }
    for (let i = 0; i < SLOT_COUNT; i++) {
        if (!isAllowedSensorName(sensors[i].sensor_name)) {
            sensors[i] = {sensor_name: 'nosensor', sensor_active: false, is_sensor_version_new: sensors[i].is_sensor_version_new};
        }
        if (sensors[i].sensor_name === 'nosensor') {
            sensors[i].sensor_active = false;
        }
    }
    const copterSimEnabled = !!opts.copterSimEnabled;
    return {
        version: META_VERSION,
        simEnabled,
        extensionPackActivated,
        copterSimEnabled,
        sensors
    };
};

/**
 * Merge Robbo meta into serialized project object (after sb3.serialize).
 * @param {object} projectObj
 * @param {object|null} robboMeta - from buildRobboSimulatorMetaForSave; null skips
 */
const mergeRobboSimulatorMetaIntoProjectJson = (projectObj, robboMeta) => {
    if (!projectObj || !robboMeta) return;
    if (!projectObj.meta || typeof projectObj.meta !== 'object') {
        projectObj.meta = {};
    } else {
        projectObj.meta = Object.assign({}, projectObj.meta);
    }
    projectObj.meta.robboSimulator = robboMeta;
};

/**
 * Read raw block from parsed project JSON (before clear).
 * @param {object} projectJSON
 * @returns {object|null}
 */
const extractRawRobboSimulatorMeta = projectJSON => {
    if (!projectJSON || !projectJSON.meta || typeof projectJSON.meta !== 'object') return null;
    const block = projectJSON.meta.robboSimulator;
    if (!block || typeof block !== 'object') return null;
    return block;
};

/**
 * Parse and normalize loaded meta; returns null if missing or unusable.
 * @param {object|null} raw
 * @returns {object|null} normalized { version, simEnabled, extensionPackActivated, sensors }
 */
const parseRobboSimulatorMeta = raw => {
    if (!raw || typeof raw !== 'object') return null;
    const version = raw.version;
    if (version !== 1 && version !== '1') {
        log.warn(`robboSimulator: unsupported version ${version}, skipping`);
        return null;
    }
    const simEnabled = !!raw.simEnabled;
    const extensionPackActivated = raw.extensionPackActivated === undefined ? false : !!raw.extensionPackActivated;
    const copterSimEnabled = raw.copterSimEnabled === undefined ? false : !!raw.copterSimEnabled;
    let sensorsRaw = raw.sensors;
    if (!Array.isArray(sensorsRaw)) {
        sensorsRaw = [];
    }
    const sensors = [];
    for (let i = 0; i < SLOT_COUNT; i++) {
        sensors.push(normalizeSavedSlot(sensorsRaw[i]));
    }
    return {
        version: META_VERSION,
        simEnabled,
        extensionPackActivated,
        copterSimEnabled,
        sensors
    };
};

const VirtualMachineRobboMeta = {
    META_VERSION,
    SLOT_COUNT,
    ALLOWED_ROBOT_SENSOR_NAMES,
    buildRobboSimulatorMetaForSave,
    mergeRobboSimulatorMetaIntoProjectJson,
    extractRawRobboSimulatorMeta,
    parseRobboSimulatorMeta,
    isAllowedSensorName
};

module.exports = VirtualMachineRobboMeta;

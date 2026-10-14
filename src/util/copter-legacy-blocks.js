/**
 * Old single-value copter reporters, replaced by `[property ▾] of copter` (`copter_value`):
 * opcode → its property. Projects are converted on load, so they keep working.
 */
const LEGACY_COPTER_REPORTER_PROPERTIES = {
    copter_x_coord: 'x',
    copter_y_coord: 'y',
    copter_z_coord: 'z',
    copter_yaw: 'yaw',
    copter_battery: 'battery',
    copter_speed: 'speed'
};

/**
 * `fly [direction] N meters` / `fly [direction] N seconds`, replaced by `fly [direction] N [unit ▾]`
 * (`copter_fly`): opcode → its unit and old amount input.
 */
const LEGACY_COPTER_FLY_UNITS = {
    copter_fly_distance: {unit: 'meters', input: 'METERS'},
    copter_fly_time: {unit: 'seconds', input: 'SECONDS'}
};

/**
 * Turn an old copter block with a direct replacement into the new one in place.
 * @param {object} block deserialized block (opcode, inputs, fields)
 * @returns {boolean} true when the block was converted
 */
const migrateLegacyCopterBlock = function (block) {
    const property = LEGACY_COPTER_REPORTER_PROPERTIES[block.opcode];
    if (property) {
        block.opcode = 'copter_value';
        block.fields = {PROPERTY: {name: 'PROPERTY', value: property}};
        return true;
    }
    const fly = LEGACY_COPTER_FLY_UNITS[block.opcode];
    if (fly) {
        block.opcode = 'copter_fly';
        const amount = block.inputs[fly.input];
        if (amount) {
            delete block.inputs[fly.input];
            block.inputs.AMOUNT = Object.assign({}, amount, {name: 'AMOUNT'});
        }
        block.fields = Object.assign({}, block.fields, {UNIT: {name: 'UNIT', value: fly.unit}});
        return true;
    }
    return false;
};

module.exports = {
    LEGACY_COPTER_FLY_UNITS,
    LEGACY_COPTER_REPORTER_PROPERTIES,
    migrateLegacyCopterBlock
};

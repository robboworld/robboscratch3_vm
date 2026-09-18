const Cast = require('./cast');
const formatMessage = require('format-message');

/** Localized labels for copter_direction / copter_directions (scratch-l10n + scratch_msgs). */
const COPTER_DIR_MSG = {
    forward: {id: 'COPTER_DIRECTION_FORWARD', default: 'Forward'},
    backward: {id: 'COPTER_DIRECTION_BACKWARD', default: 'Backward'},
    left: {id: 'COPTER_DIRECTION_LEFT', default: 'Left'},
    right: {id: 'COPTER_DIRECTION_RIGHT', default: 'Right'},
    up: {id: 'COPTER_DIRECTION_UP', default: 'up'},
    down: {id: 'COPTER_DIRECTION_DOWN', default: 'down'}
};

const COPTER_DIR_DEGREES = {
    forward: 0,
    left: 90,
    backward: 180,
    right: 270
};

/** Fallback when VM locale map lacks scratch_msgs (GUI merges ScratchMsgs in blocks.jsx). */
const COPTER_DIR_LABELS_BY_LOCALE = {
    ru: {
        forward: 'вперёд',
        backward: 'назад',
        left: 'налево',
        right: 'направо',
        up: 'вверх',
        down: 'вниз'
    },
    en: {
        forward: 'forward',
        backward: 'backward',
        left: 'left',
        right: 'right',
        up: 'up',
        down: 'down'
    }
};

/**
 * @param {*} raw dropdown value, localized label, or legacy direction_* key
 * @returns {'forward'|'backward'|'left'|'right'|'up'|'down'}
 */
const normalizeCopterDirectionArg = function (raw) {
    const s = Cast.toString(raw);
    const legacy = {
        direction_forward: 'forward',
        direction_backward: 'backward',
        direction_left: 'left',
        direction_right: 'right',
        direction_up: 'up',
        direction_down: 'down'
    };
    if (legacy[s]) return legacy[s];
    const legacyLabels = {
        'Вперёд': 'forward',
        'Назад': 'backward',
        'Налево': 'left',
        'Направо': 'right',
        'Вверх': 'up',
        'Вниз': 'down',
        Forward: 'forward',
        Backward: 'backward',
        Left: 'left',
        Right: 'right',
        Up: 'up',
        Down: 'down'
    };
    if (legacyLabels[s]) return legacyLabels[s];
    if (Object.prototype.hasOwnProperty.call(COPTER_DIR_DEGREES, s)) return s;
    if (s === 'up' || s === 'down') return s;
    for (const key of Object.keys(COPTER_DIR_MSG)) {
        if (s === formatMessage(COPTER_DIR_MSG[key]) || s === COPTER_DIR_MSG[key].default) {
            return key;
        }
    }
    for (const locale of Object.keys(COPTER_DIR_LABELS_BY_LOCALE)) {
        const table = COPTER_DIR_LABELS_BY_LOCALE[locale];
        for (const key of Object.keys(table)) {
            if (s === table[key]) return key;
        }
    }
    return 'forward';
};

/**
 * Language-neutral dropdown value, same scheme as robot_directions.
 * @param {*} raw
 * @returns {string} direction_forward | direction_backward | ...
 */
const copterDirectionDropdownValue = function (raw) {
    return `direction_${normalizeCopterDirectionArg(raw)}`;
};

module.exports = {
    COPTER_DIR_MSG,
    COPTER_DIR_DEGREES,
    COPTER_DIR_LABELS_BY_LOCALE,
    normalizeCopterDirectionArg,
    copterDirectionDropdownValue
};

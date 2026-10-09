/**
 * Simulator copters are sprites named "Robbo Quadcopter" (copter 1) and "Robbo Quadcopter N"
 * (copter N; duplicating the sprite in Scratch gives "Robbo Quadcopter2", which works too).
 */

const COPTER_SPRITE_BASE_NAME = 'Robbo Quadcopter';
const COPTER_SPRITE_NAME_PATTERN = /^Robbo Quadcopter\s*(\d*)$/;
const MAX_SIM_COPTER_NUMBER = 15;

/**
 * @param {string} name sprite name
 * @returns {?number} copter number 1..15, or null when the sprite is not a simulator copter
 */
const copterNumberFromSpriteName = function (name) {
    const match = COPTER_SPRITE_NAME_PATTERN.exec(String(name || ''));
    if (!match) return null;
    const number = match[1] ? Number(match[1]) : 1;
    return number >= 1 && number <= MAX_SIM_COPTER_NUMBER ? number : null;
};

/**
 * @param {number} number copter number
 * @returns {string} sprite name for that copter
 */
const spriteNameForCopterNumber = function (number) {
    return Number(number) === 1 ? COPTER_SPRITE_BASE_NAME : `${COPTER_SPRITE_BASE_NAME} ${number}`;
};

/**
 * @param {object} target rendered target
 * @returns {?number} copter number of an original (non-clone) simulator copter sprite
 */
const copterNumberOfTarget = function (target) {
    if (!target || !target.isOriginal || target.isStage || !target.sprite) return null;
    return copterNumberFromSpriteName(target.sprite.name);
};

/**
 * @param {Array<object>} targets runtime targets
 * @returns {Array<number>} sorted unique copter numbers present on stage
 */
const listSimCopterNumbers = function (targets) {
    const numbers = new Set();
    (targets || []).forEach(t => {
        const n = copterNumberOfTarget(t);
        if (n !== null) numbers.add(n);
    });
    return Array.from(numbers).sort((a, b) => a - b);
};

module.exports = {
    COPTER_SPRITE_BASE_NAME,
    MAX_SIM_COPTER_NUMBER,
    copterNumberFromSpriteName,
    copterNumberOfTarget,
    listSimCopterNumbers,
    spriteNameForCopterNumber
};

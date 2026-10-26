/**
 * Fix-ups of the bundled 2D simulator costumes in projects saved by older versions.
 */

/** Copter skins (128×128 px, bitmapResolution 2); assetId is the file stem. */
const COPTER_SKIN_PATTERN = /^robbo_quadcopter_topdown_v2(_spinning)?$/;

/** Older versions put the copter center at a quarter of the image: drawn 16 px off. */
const OLD_COPTER_CENTER = 32;
const COPTER_CENTER = 64;

/**
 * Moves the rotation center of an old bundled copter costume to the middle of the image.
 * A center the user moved on purpose stays.
 * @param {string} assetId costume asset id (md5 or bundled file stem)
 * @param {object} costume costume about to be loaded
 * @returns {boolean} true when the center was fixed
 */
const fixBundledCostumeCenter = function (assetId, costume) {
    if (!costume || !COPTER_SKIN_PATTERN.test(String(assetId))) return false;
    if (costume.bitmapResolution !== 2) return false;
    if (costume.rotationCenterX !== OLD_COPTER_CENTER || costume.rotationCenterY !== OLD_COPTER_CENTER) return false;
    costume.rotationCenterX = COPTER_CENTER;
    costume.rotationCenterY = COPTER_CENTER;
    return true;
};

module.exports = {
    fixBundledCostumeCenter
};

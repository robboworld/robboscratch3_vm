const Cast = require('../util/cast');
const MathUtil = require('../util/math-util');
const Timer = require('../util/timer');
const QuadcopterCommandCoordinator = require('./quadcopter-command-coordinator');
const Thread = require('../engine/thread');
const formatMessage = require('format-message');
const {COPTER_DIR_DEGREES, normalizeCopterDirectionArg} = require('../util/copter-direction-arg');

/** Turn side labels for copter_rotate / copter_turn_sides (scratch-l10n + fallback). */
const COPTER_TURN_SIDE_MSG = {
    left: {id: 'COPTER_TURN_SIDE_LEFT', default: 'left'},
    right: {id: 'COPTER_TURN_SIDE_RIGHT', default: 'right'}
};

const COPTER_TURN_SIDE_LABELS_BY_LOCALE = {
    ru: {
        left: 'влево',
        right: 'вправо'
    },
    en: {
        left: 'left',
        right: 'right'
    }
};


const {copterNumberOfTarget, listSimCopterNumbers} = require('../util/copter-sim-sprite');
const COSTUME_IDLE = 0;
const COSTUME_FLYING = 1;
const SIM_SCALE = 100;       // 1 meter = 100 Scratch pixels
/** Sprite grows by this fraction of its ground size per meter of altitude (×1.8 at 1 m). */
const SIM_SIZE_GAIN_PER_METER = 0.8;
const SIM_STEP_MS = 50;      // physics tick interval
/** Default block speed (m/s); matches the `set speed` shadow value in the toolbox. */
const DEFAULT_SPEED_MPS = 0.5;
/** Takeoff climbs a bit faster and landing descends slower than the horizontal speed. */
const SIM_TAKEOFF_SPEED_FACTOR = 1.1;
/**
 * «Take off» climbs to this absolute height and does nothing in the air (Bitcraze HL takeoff is
 * absolute): repeated blocks used to stack +0.3 m each and drove the copter up to 1.6 m.
 */
const TAKEOFF_HEIGHT_M = 0.3;
const SIM_LAND_SPEED_FACTOR = 0.44;
/** Yaw rate at {@link DEFAULT_SPEED_MPS}; scales with speed. */
const BASE_YAW_RATE_DPS = 90;
/** Speed clamp on hardware (m/s): indoor flight with Flow deck. */
const HARDWARE_MIN_SPEED_MPS = 0.1;
const HARDWARE_MAX_SPEED_MPS = 1.0;
/** Simulator battery: a full charge hovers this long (stock CF2.1, same as the DCA battery policy). */
const SIM_FULL_BATTERY_SECONDS = 6 * 60;
/** Moving and climbing drain the simulator battery faster than hovering. */
const SIM_MOVING_DRAIN_FACTOR = 1.2;
const SIM_BATTERY_TICK_MS = 250;
/** Target height at or below this is a landing (sim and hardware). */
const GROUND_TARGET_Z_M = 0.05;
/** Height of the approach before landing at a ground-level target (m). */
const HARDWARE_PRE_LAND_HEIGHT_M = 0.15;
const SIM_POS_TOLERANCE = 0.004;
const SIM_YAW_TOLERANCE = 2.5;

/**
 * Полёт по железу: каждый блок — одна HL-команда (takeoff / go_to relative / land). Траекторию
 * ведёт планировщик на борту и потом сам держит точку: поток уставок не нужен, потеря связи
 * моторы не отключает. Смещение relative go_to — в **мировой** системе, поворот — смещение yaw.
 */
const HARDWARE_YAW_RATE_DPS = 72;
const HARDWARE_MAX_YAW_RATE_DPS = 144;
/** Ожидание isDone по телеметрии — верхняя граница (секунды полёта редко дольше). */
const HARDWARE_TARGET_COMMAND_TIMEOUT_MS = 45000;
const HARDWARE_LONG_TARGET_COMMAND_TIMEOUT_MS = 120000;
/** HL go_to / distance: finish early when within this 3D error (m). */
const HARDWARE_POS_TOL_M = 0.07;
/** HL rotation: finish early when the heading is within this error (degrees). */
const HARDWARE_YAW_TOL_DEG = 5;
/** Upper bound on block wait beyond nominal HL trajectory (radio + planner). */
const HARDWARE_HL_EXTRA_WAIT_MS = 650;
/** Blocks that still run with a critical battery: they bring the copter down. */
const BATTERY_EXEMPT_COMMAND_KEYS = new Set(['copter_land']);
/** Altitude above which hover-hold is safe on project stop (matches {@link copter_is_flying}). */
const HARDWARE_GROUND_Z_M = 0.05;

/**
 * State and behaviour of one copter (simulator sprite and/or real Crazyflie). The block package
 * (`scratch3_quadcopter.js`) keeps one controller per copter number and routes each block to the
 * controller selected by the `copters [N] do` C-block.
 */
class CopterController {
    /**
     * @param {Runtime} runtime VM runtime
     * @param {{copterId?: number|string, qca?: object}} [options] copterId: 1..15 or 'new' (factory
     *   address); qca: QCA API bound to that copter (`QCA.copter(id)`), defaults to `runtime.QCA`
     */
    constructor(runtime, options) {
        /**
         * The runtime instantiating this block package.
         * @type {Runtime}
         */
        this.runtime = runtime;
        const opts = options || {};
        this.copterId = opts.copterId !== undefined ? opts.copterId : 1;
        /** Simulator sprite number of this copter (the factory copter maps to sprite 1). */
        this.simNumber = typeof this.copterId === 'number' ? this.copterId : 1;
        this._qca = opts.qca || null;
        this.x = this.QCA.get_coord("X");
        this.nowx = 0;
        this.y = this.QCA.get_coord("Y");
        this.nowy = 0;
        this.z = this.QCA.get_coord("Z");
        this.nowz = 0;
        this.defz = 0.3;
        this.yaw = this.QCA.get_coord("W");
        this.noww = 0;
        this.dir = 0;
        this.fack = 0;
        this.delta = 0.01;
        this.speed = DEFAULT_SPEED_MPS;

        this.yielded_time_start = Date.now();
        this.yielded_time_now = Date.now();
        this.yielded_max_time = 1 * 60 * 1000;

        this.x_telemetry_delta = 0;
        this.y_telemetry_delta = 0;

        // --- Simulator state ---
        /**
         * Start point of this copter on stage (m). Like a Crazyflie with a Flow deck, coordinates
         * (`sim_x`, `sim_y`) are local to it: "fly to (0.5, 0)" for every copter shifts the whole
         * formation. Dragging the copter on the ground moves its start point.
         */
        this.sim_origin_x = 0;
        this.sim_origin_y = 0;
        this.sim_x = 0;
        this.sim_y = 0;
        this.sim_z = 0;
        this.sim_yaw = 0;
        this.sim_is_flying = false;
        this.sim_battery = 100;
        this._simBatteryTimer = null;
        this.sim_interval = null;
        this._sim_rotate_remaining = 0;
        /** Sprite size (%) on the ground; captured from the user's sprite, never forced. */
        this.sim_base_size = null;
        this._simLastAppliedSize = null;
        this._simLastAppliedZ = 0;

        this.SendCordInterval = null;
        this.CopterLANDING = null;
        this._simTimeouts = new Set();
        this.commandCoordinator = new QuadcopterCommandCoordinator({
            isThreadAlive: thread => thread.status !== Thread.STATUS_DONE &&
                (!this.runtime.threads || this.runtime.threads.indexOf(thread) !== -1),
            onFlightCleanup: () => {
                if (!this.runtime || !this.QCA) return;
                // Stop where it is: the planner holds the copter (HL STOP would cut the motors).
                if (typeof this.QCA.holdPosition === 'function') {
                    this.QCA.holdPosition();
                } else if (typeof this.QCA.hoverStop === 'function') {
                    this.QCA.hoverStop();
                }
            }
        });

        this.runtime.on('PROJECT_STOP_ALL', this._onProjectStopAll.bind(this));
        this.runtime.on('ROBBO_SIM_SPRITES_INVALIDATED', this._onRobboSimSpritesInvalidated.bind(this));
    }

    /** QCA API of this copter (falls back to the runtime's QCA for the primary copter). */
    get QCA() {
        return this._qca || this.runtime.QCA;
    }

    // ===== Simulator helpers =====

    _getSimCopterTarget() {
        if (!this.runtime || !this.runtime.targets) return null;
        for (let i = 0; i < this.runtime.targets.length; i++) {
            const t = this.runtime.targets[i];
            if (copterNumberOfTarget(t) === this.simNumber) {
                return t;
            }
        }
        return null;
    }

    _disableSimByMissingSprite() {
        if (!this.runtime || !this.runtime.sim_copter_ac) return;
        // Other simulator copters keep the simulator on; only this copter's commands do nothing.
        if (listSimCopterNumbers(this.runtime.targets).length > 0) return;
        this.runtime.sim_copter_ac = false;
        this._simClearInterval();
        this._simClearAllTimeouts();
        this.fack = 0;
        this.runtime.emit('ROBBO_SIM_SPRITES_INVALIDATED', { robot: false, copter: true });
    }

    _simApplyState() {
        const target = this._getSimCopterTarget();
        if (!target) {
            this._disableSimByMissingSprite();
            return;
        }
        if (!target.draggable) {
            target.setDraggable(true);
        }
        const costumeIdx = this.sim_is_flying ? COSTUME_FLYING : COSTUME_IDLE;
        if (target.currentCostume !== costumeIdx) {
            target.setCostume(costumeIdx);
        }
        target.setDirection(this.sim_yaw);
        this._simApplySize(target);
        this._simApplyPosition(target);
        this._simSyncBatteryTimer();
    }

    /** The battery drains only while the motors spin. */
    _simSyncBatteryTimer() {
        if (this.sim_is_flying && !this._simBatteryTimer) {
            this._simBatteryLastTick = Date.now();
            this._simBatteryTimer = setInterval(() => this._simBatteryTick(), SIM_BATTERY_TICK_MS);
        } else if (!this.sim_is_flying && this._simBatteryTimer) {
            clearInterval(this._simBatteryTimer);
            this._simBatteryTimer = null;
        }
    }

    _simBatteryTick() {
        const now = Date.now();
        const dt = (now - this._simBatteryLastTick) / 1000;
        this._simBatteryLastTick = now;
        if (!this.runtime.sim_copter_ac || !this.sim_is_flying) {
            this._simSyncBatteryTimer();
            return;
        }
        const factor = this.sim_interval ? SIM_MOVING_DRAIN_FACTOR : 1;
        this.sim_battery = Math.max(0, this.sim_battery - (dt * factor * 100 / SIM_FULL_BATTERY_SECONDS));
        if (this.sim_battery > 0) return;
        // Empty: let the current command finish, then descend; new flight blocks are refused.
        if (this.sim_z <= 0.02) {
            this._simEnsureFlyingFlagByAltitude();
            this._simApplyState();
        } else if (!this.sim_interval) {
            this._simForcedLanding = true;
            this._simStartMoveToCoord(this.sim_x, this.sim_y, 0, this.sim_yaw, {zMps: this._simLandMps()});
        }
    }

    /** Simulator battery level 0..100 (%). */
    simChargeTo(percent) {
        this.sim_battery = MathUtil.clamp(Cast.toNumber(percent), 0, 100);
    }

    /**
     * Altitude scales the sprite relative to its ground size. A size set by the user (block or
     * sprite info) is detected as a mismatch with what we applied last and becomes the new base.
     */
    _simApplySize(target) {
        const lastFactor = 1 + SIM_SIZE_GAIN_PER_METER * Math.max(0, this._simLastAppliedZ);
        if (this.sim_base_size === null || target.size !== this._simLastAppliedSize) {
            this.sim_base_size = target.size / lastFactor;
        }
        const factor = 1 + SIM_SIZE_GAIN_PER_METER * Math.max(0, this.sim_z);
        target.setSize(this.sim_base_size * factor);
        this._simLastAppliedSize = target.size;
        this._simLastAppliedZ = this.sim_z;
    }

    /** Keep the visible copter on stage; clamped position is written back to the sim state. */
    _simApplyPosition(target) {
        let halfW = 0;
        let halfH = 0;
        const bounds = target.getBounds();
        if (bounds) {
            halfW = (bounds.right - bounds.left) / 2;
            halfH = (bounds.top - bounds.bottom) / 2;
        }
        // Runtime class (not required here: engine/runtime requires this module).
        const RuntimeClass = this.runtime.constructor;
        const stageW = RuntimeClass.STAGE_WIDTH || 480;
        const stageH = RuntimeClass.STAGE_HEIGHT || 360;
        const maxX = Math.max(0, (stageW / 2) - halfW) / SIM_SCALE;
        const maxY = Math.max(0, (stageH / 2) - halfH) / SIM_SCALE;
        const worldX = MathUtil.clamp(this.sim_origin_x + this.sim_x, -maxX, maxX);
        const worldY = MathUtil.clamp(this.sim_origin_y + this.sim_y, -maxY, maxY);
        this.sim_x = worldX - this.sim_origin_x;
        this.sim_y = worldY - this.sim_origin_y;
        target.setXY(worldX * SIM_SCALE, worldY * SIM_SCALE);
    }

    /** Block speed in m/s (sim and hardware share it). */
    _speedMps() {
        const v = Math.abs(Number(this.speed));
        return Number.isFinite(v) && v > 0 ? v : DEFAULT_SPEED_MPS;
    }

    _simTakeoffMps() {
        return this._speedMps() * SIM_TAKEOFF_SPEED_FACTOR;
    }

    _simLandMps() {
        return this._speedMps() * SIM_LAND_SPEED_FACTOR;
    }

    _yawRateDps() {
        return BASE_YAW_RATE_DPS * this._speedMps() / DEFAULT_SPEED_MPS;
    }

    /** Hardware speed: block speed clamped to a safe indoor range. */
    _hardwareSpeedMps() {
        return MathUtil.clamp(this._speedMps(), HARDWARE_MIN_SPEED_MPS, HARDWARE_MAX_SPEED_MPS);
    }

    _simCanExecuteAirCommand() {
        return this.sim_is_flying === true && this.sim_z > 0.02;
    }

    _simEnsureFlyingFlagByAltitude() {
        if (this.sim_z <= 0.02) {
            this.sim_z = 0;
            this.sim_is_flying = false;
            return;
        }
        this.sim_is_flying = true;
    }

    syncFromSpritePosition() {
        if (!this.runtime || !this.runtime.sim_copter_ac) return false;
        const target = this._getSimCopterTarget();
        if (!target) {
            this._disableSimByMissingSprite();
            return false;
        }
        let changed = false;
        const worldX = Number((target.x / SIM_SCALE).toFixed(3));
        const worldY = Number((target.y / SIM_SCALE).toFixed(3));
        const localX = worldX - this.sim_origin_x;
        const localY = worldY - this.sim_origin_y;
        if (Math.abs(localX - this.sim_x) > 0.0005 || Math.abs(localY - this.sim_y) > 0.0005) {
            if (!this.sim_is_flying && !this._isSimMotionActive()) {
                // On the ground the copter was carried to a new start point: local coordinates stay.
                this.sim_origin_x = worldX - this.sim_x;
                this.sim_origin_y = worldY - this.sim_y;
            } else {
                this.sim_x = localX;
                this.sim_y = localY;
            }
            changed = true;
        }
        const dir = Number(target.direction);
        if (Number.isFinite(dir)) {
            const nextYaw = this._castYawTo360(dir);
            let dyaw = Math.abs(nextYaw - this.sim_yaw);
            if (dyaw > 180) dyaw = 360 - dyaw;
            if (dyaw > 0.05) {
                this.sim_yaw = nextYaw;
                changed = true;
            }
        }
        return changed;
    }

    _isSimMotionActive() {
        return !!this.sim_interval;
    }

    _shouldSyncFromSpritePosition() {
        if (!this.runtime || !this.runtime.sim_copter_ac) return false;
        const target = this._getSimCopterTarget();
        if (!target) return false;
        if (this._isSimMotionActive()) return true;
        const localX = Number((target.x / SIM_SCALE).toFixed(3)) - this.sim_origin_x;
        const localY = Number((target.y / SIM_SCALE).toFixed(3)) - this.sim_origin_y;
        if (Math.abs(localX - this.sim_x) > 0.0005 || Math.abs(localY - this.sim_y) > 0.0005) {
            return true;
        }
        const dir = Number(target.direction);
        if (!Number.isFinite(dir)) return false;
        const nextYaw = this._castYawTo360(dir);
        let dyaw = Math.abs(nextYaw - this.sim_yaw);
        if (dyaw > 180) dyaw = 360 - dyaw;
        return dyaw > 0.05;
    }

    _syncFromSpritePositionIfNeeded() {
        if (!this._shouldSyncFromSpritePosition()) return false;
        return this.syncFromSpritePosition();
    }

    setStateFromPaletteInput(nextState) {
        if (!this.runtime || !this.runtime.sim_copter_ac) return false;
        if (!nextState || typeof nextState !== 'object') return false;
        let changed = false;
        const applyNumber = (fieldName, transform) => {
            if (nextState[fieldName] === undefined) return;
            const raw = Number(nextState[fieldName]);
            if (!Number.isFinite(raw)) return;
            this[fieldName] = transform ? transform(raw) : raw;
            changed = true;
        };
        applyNumber('sim_x');
        applyNumber('sim_y');
        applyNumber('sim_z', value => Math.max(0, value));
        applyNumber('sim_yaw', value => this._castYawTo360(value));
        if (!changed) return false;
        this._simClearInterval();
        this._simClearAllTimeouts();
        this._simEnsureFlyingFlagByAltitude();
        this.fack = 0;
        this._simApplyState();
        return true;
    }

    _simClearInterval() {
        if (this.sim_interval) {
            clearInterval(this.sim_interval);
            this.sim_interval = null;
        }
    }

    _simAddTimeout(fn, ms) {
        const id = setTimeout(() => {
            this._simTimeouts.delete(id);
            fn();
        }, ms);
        this._simTimeouts.add(id);
        return id;
    }

    _simClearAllTimeouts() {
        for (const id of this._simTimeouts) {
            clearTimeout(id);
        }
        this._simTimeouts.clear();
    }

    /**
     * greenFlag() always calls stopAll() first. On a grounded copter, hoverStop and
     * notify_setpoint_stop both disturb the commander; skip radio commands when idle on deck.
     * @returns {'hover'|'stop'|'none'}
     */
    _getHardwareStopActionOnProjectStop() {
        if (!this.runtime || !this.QCA) return 'none';
        if (typeof this.QCA.isQuadcopterConnected !== 'function' ||
            !this.QCA.isQuadcopterConnected()) {
            return 'none';
        }

        const z = Number(this.QCA.get_coord('Z'));
        const inAir = Number.isFinite(z) && z > HARDWARE_GROUND_Z_M;
        const streaming = typeof this.QCA.isStreamingSetpoints === 'function' &&
            this.QCA.isStreamingSetpoints();

        if (streaming) {
            return inAir ? 'hover' : 'stop';
        }

        if (!inAir) {
            return 'none';
        }

        const snap = typeof this.QCA.getStatusSnapshot === 'function'
            ? this.QCA.getStatusSnapshot()
            : null;
        if (snap && snap.flightState === 'landing') {
            return 'stop';
        }

        return 'hover';
    }

    _onProjectStopAll() {
        this._simClearInterval();
        this._simClearAllTimeouts();
        const stopAction = this._getHardwareStopActionOnProjectStop();
        this.commandCoordinator.cancel('projectStopAll');
        this._clearHardwareCommandIntervals();
        try {
            if (this.runtime && this.QCA) {
                if (typeof this.QCA.isQuadcopterConnected === 'function' &&
                    this.QCA.isQuadcopterConnected()) {
                    if (stopAction === 'hover' &&
                        typeof this.QCA.hoverStop === 'function') {
                        this.QCA.hoverStop();
                    } else if (stopAction === 'stop' &&
                        typeof this.QCA.stopCommands === 'function') {
                        this.QCA.stopCommands('projectStopAll');
                    }
                } else if (typeof this.QCA.stopCommands === 'function') {
                    this.QCA.stopCommands('projectStopAll');
                } else if (typeof this.QCA.disconnect === 'function') {
                    this.QCA.disconnect('projectStopAll');
                }
            }
        } catch (e) {
            // Best-effort: stop should never throw.
        }
        this.fack = 0;
    }

    /**
     * When the copter sim sprite is removed while sim was active, clear sim timers.
     * @param {Object} payload
     * @param {boolean} [payload.robot]
     * @param {boolean} [payload.copter]
     */
    _onRobboSimSpritesInvalidated(payload) {
        if (payload && payload.copter) {
            this._onProjectStopAll();
        }
    }

    /**
     * @param {object} util
     * @returns {boolean} true when copter sprite exists (or sim off)
     */
    _ensureSimCopterSprite(util) {
        if (!this.runtime.sim_copter_ac) return true;
        if (this._getSimCopterTarget()) return true;
        this._disableSimByMissingSprite();
        return false;
    }

    /**
     * Animate sim copter toward target coords. Resolves when close enough or timeout.
     * Uses the same fack/yield pattern as the hardware path.
     * @param {number} tx
     * @param {number} ty
     * @param {number} tz
     * @param {number} tyaw
     * @param {Object=} opts Optional speed overrides in m/s: xyMps, zMps.
     */
    _simStartMoveToCoord(tx, ty, tz, tyaw, opts) {
        this._simClearInterval();
        const xyMps = opts && Number.isFinite(opts.xyMps) ? opts.xyMps : this._speedMps();
        const zMps = opts && Number.isFinite(opts.zMps) ? opts.zMps : this._speedMps();
        const yawDps = this._yawRateDps();
        this.sim_interval = setInterval(() => {
            const dx = tx - this.sim_x;
            const dy = ty - this.sim_y;
            const dz = tz - this.sim_z;
            const xyStep = xyMps * (SIM_STEP_MS / 1000);
            const zStep = zMps * (SIM_STEP_MS / 1000);
            const dist2d = Math.sqrt(dx * dx + dy * dy);
            if (dist2d > xyStep) {
                this.sim_x += (dx / dist2d) * xyStep;
                this.sim_y += (dy / dist2d) * xyStep;
            } else {
                this.sim_x = tx;
                this.sim_y = ty;
            }
            if (Math.abs(dz) > zStep) {
                this.sim_z += Math.sign(dz) * zStep;
            } else {
                this.sim_z = tz;
            }
            // yaw interpolation
            let dyaw = tyaw - this.sim_yaw;
            if (dyaw > 180) dyaw -= 360;
            if (dyaw < -180) dyaw += 360;
            const yawStep = yawDps * (SIM_STEP_MS / 1000);
            if (Math.abs(dyaw) > yawStep) {
                this.sim_yaw += Math.sign(dyaw) * yawStep;
            } else {
                this.sim_yaw = tyaw;
            }
            this.sim_yaw = this._castYawTo360(this.sim_yaw);
            if (tz <= GROUND_TARGET_Z_M && this.sim_z <= GROUND_TARGET_Z_M) {
                this._simEnsureFlyingFlagByAltitude();
                if (this._simForcedLanding) {
                    // No block owns the battery landing: stop the animation on touchdown.
                    this._simForcedLanding = false;
                    this._simClearInterval();
                }
            }
            this._simApplyState();
        }, SIM_STEP_MS);
    }

    /**
     * Simulator yaw: rotate by the full signed angle (may exceed 360°), not shortest path.
     * @param {number} signedDeg
     */
    _simStartRotateBy(signedDeg) {
        this._simClearInterval();
        const remaining = Number.isFinite(signedDeg) ? Math.abs(signedDeg) : 0;
        this._sim_rotate_remaining = remaining;
        if (remaining < SIM_YAW_TOLERANCE) {
            this._sim_rotate_remaining = 0;
            return;
        }
        const sign = signedDeg >= 0 ? 1 : -1;
        const yawDps = this._yawRateDps();
        this.sim_interval = setInterval(() => {
            const yawStep = yawDps * (SIM_STEP_MS / 1000);
            const step = Math.min(this._sim_rotate_remaining, yawStep);
            this.sim_yaw = this._castYawTo360(this.sim_yaw + sign * step);
            this._sim_rotate_remaining -= step;
            if (this._sim_rotate_remaining <= SIM_YAW_TOLERANCE) {
                this._sim_rotate_remaining = 0;
                this._simClearInterval();
            }
            this._simApplyState();
        }, SIM_STEP_MS);
    }

    _simIsRotateByComplete() {
        return this._sim_rotate_remaining <= SIM_YAW_TOLERANCE && !this.sim_interval;
    }

    _simIsAtTarget(tx, ty, tz, tyaw) {
        let dyaw = Math.abs(tyaw - this.sim_yaw);
        if (dyaw > 180) dyaw = 360 - dyaw;
        return Math.abs(this.sim_x - tx) < SIM_POS_TOLERANCE &&
            Math.abs(this.sim_y - ty) < SIM_POS_TOLERANCE &&
            Math.abs(this.sim_z - tz) < SIM_POS_TOLERANCE &&
            dyaw < SIM_YAW_TOLERANCE;
    }

    _castYawTo360(yaw) {
        if (yaw > 0) {
            yaw = yaw - ((yaw / 360) >> 0) * 360;
        } else if (yaw < 0) {
            yaw = yaw + ((yaw / 360) >> 0) * -360;
        }
        return yaw;
    }

    _shortestYawDeltaDeg(fromDeg, toDeg) {
        const from = Number(fromDeg);
        const to = Number(toDeg);
        if (!Number.isFinite(from) || !Number.isFinite(to)) return 0;
        let d = (to - from) % 360;
        if (d > 180) d -= 360;
        if (d < -180) d += 360;
        return d;
    }

    /** Body-relative direction (m) → offset for HL go_to(relative). */
    _hardwareBodyOffsetMeters(meters, dirDeg) {
        const rad = Number(dirDeg) * Math.PI / 180;
        const m = Number(meters);
        return {
            bdx: m * Math.cos(rad),
            bdy: m * Math.sin(rad)
        };
    }

    /** Body XY offset rotated into Crazyflie world frame (m). */
    _hardwareWorldDeltaFromBody(bdx, bdy, yawDeg) {
        const yawRad = Number(yawDeg) * Math.PI / 180;
        const c = Math.cos(yawRad);
        const s = Math.sin(yawRad);
        return {
            dx: bdx * c - bdy * s,
            dy: bdx * s + bdy * c
        };
    }

    /**
     * Critical battery, or on the ground with too little charge for a flight: a script still running
     * after a low-battery landing must not lift the copter again. Land stays available.
     * @param {string} commandKey block opcode
     * @returns {boolean} true when the command must be skipped
     */
    _refuseForCriticalBattery(commandKey) {
        if (BATTERY_EXEMPT_COMMAND_KEYS.has(commandKey)) return false;
        if (!this.QCA || typeof this.QCA.getBatteryState !== 'function') return false;
        const state = this.QCA.getBatteryState();
        if (!state) return false;
        if (state.level === 'critical') return true;
        return state.canTakeOff === false && !this.copter_is_flying();
    }

    _clearHardwareCommandIntervals() {
        if (this.SendCordInterval) {
            clearInterval(this.SendCordInterval);
            this.SendCordInterval = null;
        }
        if (this.CopterLANDING) {
            clearInterval(this.CopterLANDING);
            this.CopterLANDING = null;
        }
    }

    /**
     * The battery is checked when a block starts. A block already running when the battery turns
     * critical is dropped: the DCA is landing the copter, and the block must not fly it on.
     * @param {string} commandKey block opcode
     * @param {object} util block util of the calling script
     * @returns {boolean} true when the block must not continue
     */
    _stopForBattery(commandKey, util) {
        if (!this.commandCoordinator.isRunning(commandKey, util)) {
            return this._refuseForCriticalBattery(commandKey);
        }
        if (BATTERY_EXEMPT_COMMAND_KEYS.has(commandKey)) return false;
        const state = this.QCA && typeof this.QCA.getBatteryState === 'function' ?
            this.QCA.getBatteryState() : null;
        if (state && state.level === 'critical') {
            this.commandCoordinator.cancel('battery');
            return true;
        }
        return false;
    }

    _runHardwareTargetCommand(commandKey, util, opts) {
        if (this._stopForBattery(commandKey, util)) return;
        const ceiling = opts.timeoutMs !== undefined
            ? opts.timeoutMs
            : HARDWARE_TARGET_COMMAND_TIMEOUT_MS;
        this.commandCoordinator.runTargetCommand(commandKey, util, {
            timeoutMs: ceiling,
            preempt: opts.preempt === true,
            start: () => {
                let preparedContext = {};
                if (typeof opts.prepare === 'function') {
                    preparedContext = opts.prepare() || {};
                }
                const context = Object.assign({ disconnected: false }, preparedContext);
                context.intervalId = setInterval(() => {
                    if (!this.QCA.isQuadcopterConnected()) {
                        context.disconnected = true;
                        return;
                    }
                    opts.dispatch(context);
                }, opts.intervalMs || 100);

                if (opts.intervalSlot === 'landing') {
                    this.CopterLANDING = context.intervalId;
                } else {
                    this.SendCordInterval = context.intervalId;
                }

                return context;
            },
            shouldFinish: (context, elapsedMs) => {
                if (context.disconnected) {
                    return true;
                }
                return opts.isDone ? opts.isDone(context, elapsedMs) : false;
            },
            finish: (context, timedOut) => {
                if (context && context.intervalId) {
                    clearInterval(context.intervalId);
                }
                if (opts.intervalSlot === 'landing') {
                    this.CopterLANDING = null;
                } else {
                    this.SendCordInterval = null;
                }
                const lostOrTimeout = Boolean(timedOut || context.disconnected);
                if (lostOrTimeout) {
                    this.QCA.hoverStop();
                }
                if (typeof opts.finish === 'function') {
                    opts.finish(context, lostOrTimeout);
                }
            },
            cancel: context => {
                if (context && context.intervalId) {
                    clearInterval(context.intervalId);
                }
                if (opts.intervalSlot === 'landing') {
                    this.CopterLANDING = null;
                } else {
                    this.SendCordInterval = null;
                }
                if (typeof opts.cancel === 'function') {
                    opts.cancel(context);
                }
            }
        });
    }

    _runHardwareTimedCommand(commandKey, util, opts) {
        if (this._stopForBattery(commandKey, util)) return;
        const softCeiling = (opts.durationMs || 0) + 15000;
        const timeoutMs = opts.timeoutMs !== undefined ? opts.timeoutMs : softCeiling;
        this.commandCoordinator.runTimedCommand(commandKey, util, {
            durationMs: opts.durationMs,
            timeoutMs,
            start: () => {
                const context = {
                    durationMs: opts.durationMs,
                    disconnected: false
                };
                if (typeof opts.start === 'function') {
                    opts.start(context);
                }
                return context;
            },
            shouldFinish: context => {
                if (!this.QCA.isQuadcopterConnected()) {
                    context.disconnected = true;
                    return true;
                }
                return false;
            },
            finish: (context, timedOut) => {
                if (typeof opts.finish === 'function') {
                    opts.finish(context, timedOut || context.disconnected);
                }
            },
            cancel: context => {
                if (typeof opts.cancel === 'function') {
                    opts.cancel(context);
                }
            }
        });
    }

    // ===== Block primitives =====

    copter_fly_up(args, util) {
        if (this.runtime.sim_copter_ac) {
            if (!this._ensureSimCopterSprite(util)) return;
            if (this.fack === 0) {
                this._syncFromSpritePositionIfNeeded();
                if (this._simCanExecuteAirCommand()) {
                    this._simApplyState();
                    return;
                }
            }
            if (this.fack === 0) {
                // As on a real copter (the estimator restarts before each takeoff), the takeoff
                // point becomes (0, 0); the sprite stays where it is.
                this.sim_origin_x += this.sim_x;
                this.sim_origin_y += this.sim_y;
                this.sim_x = 0;
                this.sim_y = 0;
                this._sim_target_z = TAKEOFF_HEIGHT_M;
                this.sim_is_flying = true;
                this._simStartMoveToCoord(this.sim_x, this.sim_y, this._sim_target_z, this.sim_yaw, {
                    zMps: this._simTakeoffMps()
                });
                this.yielded_time_start = Date.now();
                this.fack = 1;
                util.yield();
                return;
            } else if (this.fack !== 2) {
                if ((Date.now() - this.yielded_time_start) >= this.yielded_max_time) {
                    this.fack = 2;
                }
                if (this._simIsAtTarget(this.sim_x, this.sim_y, this._sim_target_z, this.sim_yaw)) {
                    this.fack = 2;
                }
                util.yield();
                return;
            }
            this._simClearInterval();
            this.sim_z = this._sim_target_z;
            // Reaching the ground is a landing: motors stop and `in flight` turns false.
            this._simEnsureFlyingFlagByAltitude();
            this._simApplyState();
            this.fack = 0;
            return;
        }

        // --- Hardware path: HL takeoff once (mirrors cfclient Flight Tab Take off button) ---
        if (!this.commandCoordinator.isRunning('copter_fly_up', util) && this.copter_is_flying()) {
            // Already in the air: «take off» does nothing.
            return;
        }
        return this._runHardwareTargetCommand('copter_fly_up', util, {
            timeoutMs: HARDWARE_LONG_TARGET_COMMAND_TIMEOUT_MS,
            intervalMs: 200,
            prepare: () => {
                this.init_start_coordinates();
                const climb = Math.max(0.05, TAKEOFF_HEIGHT_M - this.z);
                this.QCA.takeoff(TAKEOFF_HEIGHT_M, climb / this._hardwareSpeedMps());
                return {sentAt: Date.now()};
            },
            dispatch: () => {
                // HL takeoff sent in prepare.
            },
            isDone: context => {
                if (!this.QCA.isQuadcopterConnected()) {
                    return true;
                }
                // The DCA holds the flight gate until the takeoff reached its height (or gave up);
                // right after sending the takeoff it may not have taken the gate yet.
                if (Date.now() - context.sentAt < 300) {
                    return false;
                }
                return typeof this.QCA.isHlTrajectoryIdle !== 'function' || this.QCA.isHlTrajectoryIdle();
            },
            finish: () => {
                this.init_start_coordinates();
            }
        });
    }

    copter_land(args, util) {
        if (this.runtime.sim_copter_ac) {
            if (!this._ensureSimCopterSprite(util)) return;
            if (this.fack === 0) {
                this._syncFromSpritePositionIfNeeded();
                if (!this._simCanExecuteAirCommand()) {
                    this.sim_z = 0;
                    this.sim_is_flying = false;
                    this._simApplyState();
                    return;
                }
            }
            if (this.fack === 0) {
                this._simClearInterval();
                this.sim_interval = setInterval(() => {
                    this.sim_z -= this._simLandMps() * (SIM_STEP_MS / 1000);
                    if (this.sim_z <= 0.02) {
                        this.sim_z = 0;
                        this.sim_is_flying = false;
                        this._simClearInterval();
                        this.fack = 2;
                    }
                    this._simApplyState();
                }, SIM_STEP_MS);
                this.yielded_time_start = Date.now();
                this.fack = 1;
                util.yield();
                return;
            } else if (this.fack !== 2) {
                if ((Date.now() - this.yielded_time_start) >= this.yielded_max_time) {
                    this.fack = 2;
                    this.sim_z = 0;
                    this.sim_is_flying = false;
                    this._simClearInterval();
                    this._simApplyState();
                }
                util.yield();
                return;
            }
            this._simClearInterval();
            this._simApplyState();
            this.fack = 0;
            return;
        }

        // --- Hardware path: HL land once; link stays open (cfclient-style). ---
        return this._runHardwareTargetCommand('copter_land', util, {
            timeoutMs: HARDWARE_LONG_TARGET_COMMAND_TIMEOUT_MS,
            intervalMs: 200,
            intervalSlot: 'landing',
            // «Land» from another script interrupts the running move (e.g. a key while a loop flies).
            preempt: true,
            prepare: () => {
                this.init_start_coordinates();
                const landOpts = { velocityMps: this._hardwareSpeedMps() };
                const estimateMs = typeof this.QCA.computeHlLandWaitMs === 'function'
                    ? this.QCA.computeHlLandWaitMs(landOpts)
                    : 4000;
                this.QCA.landAndClose(landOpts);
                return {
                    startedAt: Date.now(),
                    landBlockDeadlineMs: Date.now() + estimateMs + 2000
                };
            },
            dispatch: () => {
                // HL land dispatched in prepare — avoid 200ms gap before handoff.
            },
            isDone: context => {
                if (!this.QCA.isQuadcopterConnected()) {
                    return true;
                }
                // Done when the copter is down; the deadline only bounds a lost or stuck landing.
                const down = Date.now() - context.startedAt >= 500 && !this.copter_is_flying();
                return down || Date.now() >= context.landBlockDeadlineMs;
            },
            finish: () => {
                this.init_start_coordinates();
            }
        });
    }

    copter_stop() {
        if (this.runtime.sim_copter_ac) {
            this._simClearInterval();
            this._simClearAllTimeouts();
            this.sim_is_flying = false;
            this.sim_z = 0;
            this._simApplyState();
            this.fack = 0;
            return;
        }

        // --- Hardware path: immediate motor cut / emergency (not HL land). ---
        this.commandCoordinator.cancel('manualStop');
        this._clearHardwareCommandIntervals();
        if (this.QCA && typeof this.QCA.emergencyStop === 'function') {
            this.QCA.emergencyStop({ keepConnected: true });
        } else if (this.QCA && typeof this.QCA.landAndClose === 'function') {
            this.QCA.landAndClose();
        }
    }

    _isVerticalFlightDirection(dirKey) {
        return dirKey === 'up' || dirKey === 'down';
    }

    _copterFlyDistanceVertical(dirKey, meters, util) {
        const dz = dirKey === 'up' ? Cast.toNumber(meters) : -Cast.toNumber(meters);
        if (this.runtime.sim_copter_ac) {
            if (!this._ensureSimCopterSprite(util)) return;
            if (this.fack === 0) {
                this._syncFromSpritePositionIfNeeded();
                if (!this._simCanExecuteAirCommand()) return;
            }
            if (this.fack === 0) {
                this._sim_target_z = Math.max(0, this.sim_z + dz);
                this._simStartMoveToCoord(this.sim_x, this.sim_y, this._sim_target_z, this.sim_yaw);
                this.yielded_time_start = Date.now();
                this.fack = 1;
                util.yield();
                return;
            } else if (this.fack !== 2) {
                if ((Date.now() - this.yielded_time_start) >= this.yielded_max_time) this.fack = 2;
                if (this._simIsAtTarget(this.sim_x, this.sim_y, this._sim_target_z, this.sim_yaw)) this.fack = 2;
                util.yield();
                return;
            }
            this._simClearInterval();
            this.sim_z = this._sim_target_z;
            // Reaching the ground is a landing: motors stop and `in flight` turns false.
            this._simEnsureFlyingFlagByAltitude();
            this._simApplyState();
            this.fack = 0;
            return;
        }

        // --- Hardware path: HL relative go_to; down to the floor is a landing (decided at the start). ---
        return this._runHardwareHlMove('copter_fly_distance', util, () => ({dx: 0, dy: 0, dz: dz}));
    }

    _copterFlyTimeVertical(dirKey, seconds, util) {
        const durationMs = Math.max(0, Cast.toNumber(seconds)) * 1000;
        if (durationMs <= 0) {
            return;
        }
        const sign = dirKey === 'up' ? 1 : -1;

        if (this.runtime.sim_copter_ac) {
            if (!this._ensureSimCopterSprite(util)) return;
            if (this.fack === 0) {
                this._syncFromSpritePositionIfNeeded();
                if (!this._simCanExecuteAirCommand()) return;
            }
            if (this.fack === 0) {
                const vz = sign * this._speedMps();
                this._simClearInterval();
                this.sim_interval = setInterval(() => {
                    this.sim_z += vz * (SIM_STEP_MS / 1000);
                    if (this.sim_z < 0) this.sim_z = 0;
                    this._simEnsureFlyingFlagByAltitude();
                    this._simApplyState();
                }, SIM_STEP_MS);
                this.fack = 1;
                this._simAddTimeout(() => {
                    this.fack = 2;
                }, durationMs);
                util.yield();
                return;
            } else if (this.fack !== 2) {
                util.yield();
                return;
            }
            this._simClearInterval();
            this._simApplyState();
            this.fack = 0;
            return;
        }

        // --- Hardware path: HL relative go_to over the block time. ---
        return this._runHardwareHlMove('copter_fly_time', util, () => ({
            dx: 0,
            dy: 0,
            dz: sign * this._hardwareSpeedMps() * durationMs / 1000,
            durationS: durationMs / 1000
        }));
    }

    /**
     * `fly [direction] (N) [meters ▾ / seconds ▾]`.
     * @param {object} args DIRECTION, AMOUNT, UNIT ('meters' | 'seconds')
     * @param {object} util block utility
     * @returns {*} undefined; yields while the copter flies
     */
    copter_fly(args, util) {
        if (args.UNIT === 'seconds') {
            return this._flyTime({DIRECTION: args.DIRECTION, SECONDS: args.AMOUNT}, util);
        }
        return this._flyDistance({DIRECTION: args.DIRECTION, METERS: args.AMOUNT}, util);
    }

    _flyDistance(args, util) {
        const dirKey = this._normalizeDirectionArg(args.DIRECTION);
        if (this._isVerticalFlightDirection(dirKey)) {
            return this._copterFlyDistanceVertical(dirKey, args.METERS, util);
        }
        if (this.fack === 0) {
            this.dir = COPTER_DIR_DEGREES[dirKey] || 0;
        }
        if (this.runtime.sim_copter_ac) {
            if (!this._ensureSimCopterSprite(util)) return;
            if (this.fack === 0) {
                this._syncFromSpritePositionIfNeeded();
                if (!this._simCanExecuteAirCommand()) return;
            }
            if (this.fack === 0) {
                const meters = Number(args.METERS);
                const rad = (90 - (this.sim_yaw + this.dir)) * Math.PI / 180;
                const tx = this.sim_x + meters * Math.cos(rad);
                const ty = this.sim_y + meters * Math.sin(rad);
                this._sim_target_x = tx;
                this._sim_target_y = ty;
                this._simStartMoveToCoord(tx, ty, this.sim_z, this.sim_yaw);
                this.yielded_time_start = Date.now();
                this.fack = 1;
                util.yield();
                return;
            } else if (this.fack !== 2) {
                if ((Date.now() - this.yielded_time_start) >= this.yielded_max_time) this.fack = 2;
                if (this._simIsAtTarget(this._sim_target_x, this._sim_target_y, this.sim_z, this.sim_yaw)) this.fack = 2;
                util.yield();
                return;
            }
            this._simClearInterval();
            this.sim_x = this._sim_target_x;
            this.sim_y = this._sim_target_y;
            this._simApplyState();
            this.fack = 0;
            return;
        }

        // --- Hardware path: HL relative go_to; the offset is turned from the copter's axes into the world frame. ---
        const meters = Cast.toNumber(args.METERS);
        return this._runHardwareHlMove('copter_fly_distance', util, () => this._hardwareWorldOffset(meters, this.dir));
    }

    /**
     * Offset along a body direction (degrees from the nose, CCW) turned into the world frame by the
     * current heading: relative go_to adds offsets in world coordinates.
     * @param {number} meters distance
     * @param {number} dirDeg body direction
     * @returns {{dx: number, dy: number, dz: number}} world offset (m)
     */
    _hardwareWorldOffset(meters, dirDeg) {
        const {bdx, bdy} = this._hardwareBodyOffsetMeters(meters, dirDeg);
        const {dx, dy} = this._hardwareWorldDeltaFromBody(bdx, bdy, this.yaw);
        return {dx, dy, dz: 0};
    }

    _flyTime(args, util) {
        const dirKey = this._normalizeDirectionArg(args.DIRECTION);
        if (this._isVerticalFlightDirection(dirKey)) {
            return this._copterFlyTimeVertical(dirKey, args.SECONDS, util);
        }
        if (this.fack === 0) {
            this.dir = COPTER_DIR_DEGREES[dirKey] || 0;
        }
        if (this.runtime.sim_copter_ac) {
            if (!this._ensureSimCopterSprite(util)) return;
            if (this.fack === 0) {
                this._syncFromSpritePositionIfNeeded();
                if (!this._simCanExecuteAirCommand()) return;
            }
            if (this.fack === 0) {
                const headingRad = (90 - (this.sim_yaw + this.dir)) * Math.PI / 180;
                const vx = this._speedMps() * Math.cos(headingRad);
                const vy = this._speedMps() * Math.sin(headingRad);
                this._simClearInterval();
                this.sim_interval = setInterval(() => {
                    this.sim_x += vx * (SIM_STEP_MS / 1000);
                    this.sim_y += vy * (SIM_STEP_MS / 1000);
                    this._simApplyState();
                }, SIM_STEP_MS);
                this.fack = 1;
                const time_to_fly = Number(args.SECONDS) * 1000;
                this._simAddTimeout(() => {
                    this.fack = 2;
                }, time_to_fly);
                util.yield();
                return;
            } else if (this.fack !== 2) {
                util.yield();
                return;
            }
            this._simClearInterval();
            this._simApplyState();
            this.fack = 0;
            return;
        }

        // --- Hardware path: HL relative go_to over the block time (speed × time along the direction). ---
        const seconds = Math.max(0, Cast.toNumber(args.SECONDS));
        return this._runHardwareHlMove('copter_fly_time', util, () => Object.assign(
            this._hardwareWorldOffset(this._hardwareSpeedMps() * seconds, this.dir),
            {durationS: seconds}
        ));
    }

    copter_x_coord(args, util) {
        if (this.runtime.sim_copter_ac) {
            this._syncFromSpritePositionIfNeeded();
            return Number(this.sim_x.toFixed(3));
        }
        return Number(this.QCA.telemetry_palette_get_coord("X"));
    }

    copter_y_coord(args, util) {
        if (this.runtime.sim_copter_ac) {
            this._syncFromSpritePositionIfNeeded();
            return Number(this.sim_y.toFixed(3));
        }
        return Number(this.QCA.telemetry_palette_get_coord("Y"));
    }

    copter_yaw(args, util) {
        if (this.runtime.sim_copter_ac) return Number(this.sim_yaw.toFixed(1));
        return Number(this.QCA.telemetry_palette_get_coord("W"));
    }

    copter_z_coord(args, util) {
        if (this.runtime.sim_copter_ac) return Number(this.sim_z.toFixed(3));
        return Number(this.QCA.telemetry_palette_get_coord("Z"));
    }

    /**
     * Sim path: animate toward resolved world targets (tx, ty, tz).
     * @param {number} tx
     * @param {number} ty
     * @param {number} tz
     * @param {object} util
     */
    _simFlyToResolvedTargets(tx, ty, tz, util) {
        if (!this._ensureSimCopterSprite(util)) return;
        if (this.fack === 0) {
            this._syncFromSpritePositionIfNeeded();
            if (!this._simCanExecuteAirCommand()) return;
        }
        if (this.fack === 0) {
            this._sim_target_x = tx;
            this._sim_target_y = ty;
            this._sim_target_z = tz;
            this._simStartMoveToCoord(tx, ty, tz, this.sim_yaw);
            this.yielded_time_start = Date.now();
            this.fack = 1;
            util.yield();
            return;
        } else if (this.fack !== 2) {
            if ((Date.now() - this.yielded_time_start) >= this.yielded_max_time) this.fack = 2;
            if (this._simIsAtTarget(this._sim_target_x, this._sim_target_y, this._sim_target_z, this.sim_yaw)) this.fack = 2;
            util.yield();
            return;
        }
        this._simClearInterval();
        this.sim_x = this._sim_target_x;
        this.sim_y = this._sim_target_y;
        this.sim_z = this._sim_target_z;
        this._simEnsureFlyingFlagByAltitude();
        this._simApplyState();
        this.fack = 0;
    }

    /**
     * Hardware path: fly to world coordinates without changing heading (see {@link _runHardwareHlMove}).
     * @param {number} tx world X (m, telemetry frame)
     * @param {number} ty world Y (m, telemetry frame)
     * @param {number} targetZ height (m); at ground level the copter lands there
     * @param {object} util block util
     * @param {object} [opts] commandKey: coordinator key; durationS: fixed flight time
     * @returns {undefined}
     */
    _hardwareFlyToWorld(tx, ty, targetZ, util, opts) {
        const options = opts || {};
        return this._runHardwareHlMove(options.commandKey || 'copter_fly_to_coords', util, () => {
            const dx = tx - this.x;
            const dy = ty - this.y;
            const dz = targetZ - this.z;
            return {dx, dy, dz, durationS: options.durationS};
        });
    }

    /**
     * Hardware move as one relative HL go_to: the onboard planner flies the offset and then holds
     * the point by itself. Nothing is streamed, so a lost link does not stop the motors, and the
     * height follows the command instead of the drifting estimate.
     *
     * Firmware relative go_to adds the offset in the WORLD frame (planner.c: hover_pos =
     * vadd(hover_pos, curr_pos)) and adds the yaw offset to the current heading.
     *
     * @param {string} commandKey coordinator key of the block
     * @param {object} util block util
     * @param {Function} plan returns the offset from the pose at the start, called once when the block
     *   starts: {dx, dy, dz} in m (world frame), optional dyawDeg (CCW) and durationS (defaults to
     *   path length / speed). A target at ground level is a landing.
     * @returns {undefined}
     */
    _runHardwareHlMove(commandKey, util, plan) {
        if (!this.commandCoordinator.isRunning(commandKey, util) && !this._hardwareCanMove()) {
            return;
        }
        return this._runHardwareTargetCommand(commandKey, util, {
            timeoutMs: HARDWARE_LONG_TARGET_COMMAND_TIMEOUT_MS,
            intervalMs: 100,
            prepare: () => {
                this.init_start_coordinates();
                const step = plan() || {};
                const dx = Number(step.dx) || 0;
                const dy = Number(step.dy) || 0;
                const dyawDeg = Number(step.dyawDeg) || 0;
                const wantedZ = this.z + (Number(step.dz) || 0);
                // A target on the ground is a landing: fly over the point low, then land there.
                const landAfter = wantedZ <= GROUND_TARGET_Z_M;
                const tz = landAfter ? Math.min(this.z, HARDWARE_PRE_LAND_HEIGHT_M) : wantedZ;
                const dz = tz - this.z;
                const pathLen = Math.hypot(dx, dy, dz);
                if (pathLen < 1e-6 && Math.abs(dyawDeg) < 0.5 && !landAfter) {
                    return {skip: true};
                }
                const fixed = Number(step.durationS);
                const durationS = Number.isFinite(fixed) && fixed > 0 ?
                    Math.max(0.25, fixed) :
                    Math.max(0.25, pathLen / this._hardwareSpeedMps(),
                        Math.abs(dyawDeg) / HARDWARE_YAW_RATE_DPS);
                // Sent right away: the block must not finish before the copter got its command.
                this.QCA.goToHighLevel(dx, dy, dz, dyawDeg * Math.PI / 180, durationS,
                    {relative: true, forceLegacy: true});
                return {
                    skip: false,
                    landAfter: landAfter,
                    landing: false,
                    tx: this.x + dx,
                    ty: this.y + dy,
                    tz: tz,
                    tyawDeg: this.yaw + dyawDeg,
                    deadlineMs: Date.now() + (durationS * 1000) + HARDWARE_HL_EXTRA_WAIT_MS
                };
            },
            dispatch: () => {
                // The go_to was sent in prepare; the planner flies and holds on board.
            },
            isDone: context => {
                if (context.skip) {
                    return true;
                }
                if (!this.QCA.isQuadcopterConnected()) {
                    return true;
                }
                if (context.landing) {
                    return Date.now() >= context.landDeadlineMs ||
                        (Date.now() >= context.landStartedMs + 500 && !this.copter_is_flying());
                }
                const x = Number(this.QCA.get_coord('X'));
                const y = Number(this.QCA.get_coord('Y'));
                const z = Number(this.QCA.get_coord('Z'));
                const snap = this.QCA.getTelemetrySnapshot();
                const yawDeg = Number(snap && snap.yawDeg);
                const atPoint = [x, y, z].every(Number.isFinite) &&
                    Math.hypot(x - context.tx, y - context.ty, z - context.tz) < HARDWARE_POS_TOL_M;
                const atHeading = !Number.isFinite(yawDeg) ||
                    Math.abs(this._shortestYawDeltaDeg(yawDeg, context.tyawDeg)) < HARDWARE_YAW_TOL_DEG;
                const reached = atPoint && atHeading;
                const deadlineReached = Date.now() >= context.deadlineMs;
                if ((reached || deadlineReached) && context.landAfter) {
                    const landOpts = {velocityMps: this._hardwareSpeedMps()};
                    context.landing = true;
                    context.landStartedMs = Date.now();
                    context.landDeadlineMs = Date.now() + this.QCA.computeHlLandWaitMs(landOpts) + 2000;
                    this.QCA.landAndClose(landOpts);
                    return false;
                }
                return reached || deadlineReached;
            },
            finish: () => {
                // The planner already holds the reached point: nothing to send.
                this.init_start_coordinates();
            }
        });
    }

    /**
     * A move block may start: the copter is in the air (on the ground moves do nothing, as in the
     * simulator) and is not landing on its own because of the battery or the link.
     * @returns {boolean} true when the move may start
     */
    _hardwareCanMove() {
        if (!this.copter_is_flying()) return false;
        return !(typeof this.QCA.isSafetyLanding === 'function' && this.QCA.isSafetyLanding());
    }

    copter_fly_to_coords(args, util) {
        if (this.runtime.sim_copter_ac) {
            const tx = Number(args.X_COORD);
            // Sim uses Scratch-aligned Y (same as sim_y / reporter); do not apply hardware Y flip.
            const ty = Number(args.Y_COORD);
            const tz = Math.max(0, Number(args.Z_COORD));
            return this._simFlyToResolvedTargets(tx, ty, tz, util);
        }

        this.x_telemetry_delta = this.QCA.get_x_telemetry_delta();
        this.y_telemetry_delta = this.QCA.get_y_telemetry_delta();
        this.init_start_coordinates();
        const tx = Number(args.X_COORD) + this.x_telemetry_delta;
        const ty = (Number(args.Y_COORD) * -1) + this.y_telemetry_delta;
        const tz = Number(args.Z_COORD);
        return this._hardwareFlyToWorld(tx, ty, tz, util);
    }

    copter_move_axis_to(args, util) {
        const axis = Cast.toString(args.AXIS).toUpperCase();
        const coord = Number(args.COORD);

        if (this.runtime.sim_copter_ac) {
            const tx = axis === 'X' ? coord : this.sim_x;
            // Sim uses Scratch-aligned Y (same as sim_y / reporter); do not apply hardware Y flip.
            const ty = axis === 'Y' ? coord : this.sim_y;
            const tz = axis === 'Z' ? Math.max(0, coord) : this.sim_z;
            return this._simFlyToResolvedTargets(tx, ty, tz, util);
        }

        this.x_telemetry_delta = this.QCA.get_x_telemetry_delta();
        this.y_telemetry_delta = this.QCA.get_y_telemetry_delta();
        this.init_start_coordinates();
        const tx = axis === 'X' ? coord + this.x_telemetry_delta : this.x;
        const ty = axis === 'Y' ? (-coord) + this.y_telemetry_delta : this.y;
        const tz = axis === 'Z' ? coord : this.z;
        return this._hardwareFlyToWorld(tx, ty, tz, util);
    }

    cast_yaw_to_360(yaw) {
        return this._castYawTo360(yaw);
    }

    copter_rotate(args, util) {
        const magnitude = Math.abs(Cast.toNumber(args.DEGREES));
        const side = this._normalizeTurnSide(args.SIDE);
        const signedDeg = side === 'left' ? -magnitude : magnitude;

        if (this.runtime.sim_copter_ac) {
            if (!this._ensureSimCopterSprite(util)) return;
            if (this.fack === 0) {
                this._syncFromSpritePositionIfNeeded();
                if (!this._simCanExecuteAirCommand()) return;
            }
            if (this.fack === 0) {
                this._simStartRotateBy(signedDeg);
                this.yielded_time_start = Date.now();
                this.fack = 1;
                util.yield();
                return;
            } else if (this.fack !== 2) {
                if ((Date.now() - this.yielded_time_start) >= this.yielded_max_time) this.fack = 2;
                if (this._simIsRotateByComplete()) this.fack = 2;
                util.yield();
                return;
            }
            this._simClearInterval();
            this._simApplyState();
            this.fack = 0;
            return;
        }

        // --- Hardware path: HL relative go_to with a yaw offset only (CCW positive). ---
        const yawRateDps = Math.min(
            HARDWARE_MAX_YAW_RATE_DPS,
            HARDWARE_YAW_RATE_DPS * this._hardwareSpeedMps() / DEFAULT_SPEED_MPS
        );
        return this._runHardwareHlMove('copter_rotate', util, () => ({
            dx: 0,
            dy: 0,
            dz: 0,
            dyawDeg: -signedDeg,
            durationS: magnitude / yawRateDps
        }));
    }

    /**
     * @param {*} raw dropdown value, localized label, or legacy turn-side text
     * @returns {'left'|'right'}
     */
    _normalizeTurnSide(raw) {
        if (raw === undefined || raw === null || raw === '') {
            return 'right';
        }
        const s = Cast.toString(raw);
        if (s === 'left' || s === 'right') {
            return s;
        }
        if (s === 'влево' || s === 'налево' || s === 'Налево') {
            return 'left';
        }
        if (s === 'вправо' || s === 'направо' || s === 'Направо') {
            return 'right';
        }
        for (const key of Object.keys(COPTER_TURN_SIDE_MSG)) {
            if (s === formatMessage(COPTER_TURN_SIDE_MSG[key]) || s === COPTER_TURN_SIDE_MSG[key].default) {
                return key;
            }
        }
        for (const locale of Object.keys(COPTER_TURN_SIDE_LABELS_BY_LOCALE)) {
            const table = COPTER_TURN_SIDE_LABELS_BY_LOCALE[locale];
            for (const key of Object.keys(table)) {
                if (s === table[key]) {
                    return key;
                }
            }
        }
        return 'right';
    }

    /**
     * @param {*} raw dropdown value, localized label, or legacy direction_* key
     * @returns {'forward'|'backward'|'left'|'right'|'up'|'down'}
     */
    _normalizeDirectionArg(raw) {
        return normalizeCopterDirectionArg(raw);
    }

    /** Link quality 0..100 %; the simulator link is perfect. */
    copter_link_quality() {
        if (this.runtime.sim_copter_ac) return 100;
        if (!this.QCA || typeof this.QCA.getLinkQuality !== 'function') return 0;
        if (!this.QCA.isQuadcopterConnected()) return 0;
        return Number(this.QCA.getLinkQuality()) || 0;
    }

    /** `charge copter to [N] %` — simulator only; a real battery charges on its charger. */
    copter_charge_battery(args) {
        if (this.runtime.sim_copter_ac) {
            this.simChargeTo(args.PERCENT);
        }
    }

    copter_battery() {
        if (this.runtime.sim_copter_ac) return Math.round(this.sim_battery);
        if (!this.QCA || typeof this.QCA.getTelemetrySnapshot !== 'function') return 0;
        const snap = this.QCA.getTelemetrySnapshot();
        return snap ? (Number(snap.batteryPercent) || 0) : 0;
    }

    copter_is_flying() {
        if (this.runtime.sim_copter_ac) return this.sim_is_flying === true;
        if (!this.QCA || typeof this.QCA.isQuadcopterConnected !== 'function') return false;
        if (typeof this.QCA.isQuadcopterFlying === 'function') {
            // Supervisor `is flying` when the firmware reports it, telemetry height otherwise.
            return this.QCA.isQuadcopterFlying();
        }
        if (!this.QCA.isQuadcopterConnected()) return false;
        return Number(this.QCA.get_coord('Z')) > HARDWARE_GROUND_Z_M;
    }

    copter_set_speed(args) {
        const s = Number(args.SPEED);
        if (Number.isFinite(s) && s > 0) {
            this.speed = s;
        }
    }

    copter_speed() {
        const s = Number(this.speed);
        return Number.isFinite(s) ? s : 0;
    }

    init_start_coordinates() {
        const snap = this.QCA.getTelemetrySnapshot();
        const xRaw = Number(this.QCA.get_coord("X"));
        const yRaw = Number(this.QCA.get_coord("Y"));
        const zRaw = Number(this.QCA.get_coord("Z"));
        this.yaw = Number(snap && snap.yawDeg) || 0;
        this.x = xRaw;
        this.y = yRaw;
        this.z = zRaw;
    }
}

module.exports = CopterController;

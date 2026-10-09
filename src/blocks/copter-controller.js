const Cast = require('../util/cast');
const MathUtil = require('../util/math-util');
const Timer = require('../util/timer');
const QuadcopterCommandCoordinator = require('./quadcopter-command-coordinator');
const formatMessage = require('format-message');
const {
    COPTER_DIR_MSG,
    COPTER_DIR_DEGREES,
    COPTER_DIR_LABELS_BY_LOCALE,
    normalizeCopterDirectionArg
} = require('../util/copter-direction-arg');

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
 * Полёт по железу: velocity setpoints — в **системе координат корпуса** (cflib send_hover_setpoint).
 * HL go_to(relative) — смещение в body frame. Полёт в точку по координатам — velocity, yaw rate 0.
 */
const HARDWARE_YAW_RATE_DPS = 72;
const HARDWARE_MAX_YAW_RATE_DPS = 144;
/** Ожидание isDone по телеметрии — верхняя граница (секунды полёта редко дольше). */
const HARDWARE_TARGET_COMMAND_TIMEOUT_MS = 45000;
const HARDWARE_LONG_TARGET_COMMAND_TIMEOUT_MS = 120000;
/** HL go_to / distance: finish early when within this 3D error (m). */
const HARDWARE_POS_TOL_M = 0.07;
/** Upper bound on block wait beyond nominal HL trajectory (radio + planner). */
const HARDWARE_HL_EXTRA_WAIT_MS = 650;
/** VM target commands that enqueue firmware HL trajectories (takeoff / land / go_to). */
const HL_TARGET_COMMAND_KEYS = new Set([
    'copter_fly_up',
    'copter_land',
    'copter_fly_distance',
    'copter_fly_to_coords',
    'copter_move_axis_to'
]);
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
        this.flightDirKey = 'forward';
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
            onFlightCleanup: (reason, command) => {
                if (!this.runtime || !this.QCA) return;
                const key = command && command.key;
                if (key && HL_TARGET_COMMAND_KEYS.has(key) &&
                    typeof this.QCA.abortHlTrajectories === 'function') {
                    this.QCA.abortHlTrajectories(reason);
                    return;
                }
                if (typeof this.QCA.softStopStreaming === 'function') {
                    this.QCA.softStopStreaming(reason);
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

    /** Hover setpoints jump to the target height at the firmware's pace; ramp it at block speed. */
    _startHeightRamp(startZ) {
        return { rampZ: Number(startZ) || 0, rampAt: Date.now() };
    }

    _stepHeightRamp(context, targetZ) {
        const now = Date.now();
        const dt = Math.max(0, (now - context.rampAt) / 1000);
        context.rampAt = now;
        const step = this._hardwareSpeedMps() * dt;
        const delta = targetZ - context.rampZ;
        context.rampZ = Math.abs(delta) <= step ? targetZ : context.rampZ + Math.sign(delta) * step;
        return context.rampZ;
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

    /**
     * HL takeoff/land/go_to holds the flight gate; velocity blocks must yield until it is released.
     * @param {object} util
     * @returns {boolean} true when streaming commands may start
     */
    _yieldUntilVelocityStreamingReady(util) {
        if (!this.QCA || typeof this.QCA.canAcceptVelocityStreaming !== 'function') {
            return true;
        }
        if (this.QCA.canAcceptVelocityStreaming()) {
            return true;
        }
        util.yield();
        return false;
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

    /** Body-relative direction (deg) → velocity for legacy hover/velocity setpoints (m/s). */
    _hardwareBodyVelocity(speed, dirDeg) {
        const rad = Number(dirDeg) * Math.PI / 180;
        const s = Number(speed);
        return {
            vx: s * Math.cos(rad),
            vy: s * Math.sin(rad)
        };
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
     * Critical battery: flight blocks do not start (in the air the DCA battery watchdog is already
     * landing the copter); land and stop stay available.
     * @param {string} commandKey block opcode
     * @returns {boolean} true when the command must be skipped
     */
    /**
     * Critical battery, or on the ground with too little charge for a flight: a script still running
     * after a low-battery landing must not lift the copter again.
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

    _runHardwareTargetCommand(commandKey, util, opts) {
        if (this._refuseForCriticalBattery(commandKey)) return;
        const ceiling = opts.timeoutMs !== undefined
            ? opts.timeoutMs
            : HARDWARE_TARGET_COMMAND_TIMEOUT_MS;
        this.commandCoordinator.runTargetCommand(commandKey, util, {
            timeoutMs: ceiling,
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
        if (this._refuseForCriticalBattery(commandKey)) return;
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
                this.z = TAKEOFF_HEIGHT_M;
                return { hlScheduled: false };
            },
            dispatch: (context) => {
                if (context.hlScheduled) {
                    return;
                }
                context.hlScheduled = true;
                const climb = Math.max(0.05, this.z - Number(this.QCA.get_coord('Z')));
                this.QCA.takeoff(this.z, climb / this._hardwareSpeedMps());
            },
            isDone: () => {
                if (!this.QCA.isQuadcopterConnected()) {
                    return true;
                }
                if (typeof this.QCA.isHlTrajectoryIdle === 'function' &&
                    !this.QCA.isHlTrajectoryIdle()) {
                    return false;
                }
                const nowZ = Number(this.QCA.get_coord("Z"));
                // HL takeoff may use a higher absolute target than `this.z` when telemetry Z is stale
                // (see QCA.takeoff); finish once we've reached at least the block's nominal height.
                return Number.isFinite(nowZ) && nowZ >= this.z - this.delta;
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
            prepare: () => {
                this.init_start_coordinates();
                const landOpts = { velocityMps: this._hardwareSpeedMps() };
                const estimateMs = typeof this.QCA.computeHlLandWaitMs === 'function'
                    ? this.QCA.computeHlLandWaitMs(landOpts)
                    : 4000;
                this.QCA.landAndClose(landOpts);
                return {
                    hlScheduled: true,
                    landBlockDeadlineMs: Date.now() + estimateMs
                };
            },
            dispatch: () => {
                // HL land dispatched in prepare — avoid 200ms gap before handoff.
            },
            isDone: (context, elapsedMs) => {
                if (!this.QCA.isQuadcopterConnected()) {
                    return true;
                }
                if (typeof context.landBlockDeadlineMs === 'number') {
                    return Date.now() >= context.landBlockDeadlineMs;
                }
                return elapsedMs >= 4000;
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

    copter_status(args, util) {
        if (this.runtime.sim_copter_ac) {
            if (!this._getSimCopterTarget()) {
                this._disableSimByMissingSprite();
                return false;
            }
            return true;
        }
        return (this.QCA.isQuadcopterConnected());
    }

    _isVerticalFlightDirection(dirKey) {
        return dirKey === 'up' || dirKey === 'down';
    }

    _resolveFlightDirectionFromArgs(args) {
        if (args.DIRECTION !== undefined) {
            return this._normalizeDirectionArg(args.DIRECTION);
        }
        return this.flightDirKey || 'forward';
    }

    _copterFlyDistanceVertical(dirKey, meters, util) {
        const dz = dirKey === 'up' ? Number(meters) : -Number(meters);
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

        if (Math.abs(dz) < 1e-6) {
            return this._runHardwareTimedCommand('copter_fly_distance', util, {
                durationMs: 1,
                start: () => { },
                finish: () => this.init_start_coordinates()
            });
        }

        if (Number(this.QCA.get_coord('Z')) + dz <= GROUND_TARGET_Z_M) {
            return this.copter_land({}, util);
        }
        return this._runHardwareTargetCommand('copter_fly_distance', util, {
            prepare: () => {
                this.init_start_coordinates();
                const startZ = this.z;
                this.z = Math.max(0, this.z + dz);
                return this._startHeightRamp(startZ);
            },
            dispatch: context => {
                this.QCA.move_with_speed(0, 0, 0, this._stepHeightRamp(context, this.z));
            },
            isDone: () => {
                this.nowz = Number(this.QCA.get_coord('Z'));
                return Math.abs(this.nowz - this.z) < this.delta;
            },
            finish: () => {
                this.QCA.hoverStop();
                this.init_start_coordinates();
            }
        });
    }

    _copterFlyTimeVertical(dirKey, seconds, util) {
        const durationMs = Math.max(0, Number(seconds)) * 1000;
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

        return this._runHardwareTimedCommand('copter_fly_time', util, {
            durationMs,
            start: () => {
                this.init_start_coordinates();
                this.z = Number(this.QCA.get_coord('Z')) + sign * this._hardwareSpeedMps() * Number(seconds);
                if (this.z < 0) this.z = 0;
                this.QCA.move_with_speed(0, 0, 0, this.z);
            },
            finish: () => {
                this.QCA.hoverStop();
                this.init_start_coordinates();
            }
        });
    }

    copter_fly_distance(args, util) {
        if (this.fack === 0 && args.DIRECTION !== undefined) {
            const dirKey = this._normalizeDirectionArg(args.DIRECTION);
            if (this._isVerticalFlightDirection(dirKey)) {
                this.flightDirKey = dirKey;
            } else {
                this._applyDirectionKey(dirKey);
            }
        }
        const activeDirKey = this._resolveFlightDirectionFromArgs(args);
        if (this._isVerticalFlightDirection(activeDirKey)) {
            return this._copterFlyDistanceVertical(activeDirKey, args.METERS, util);
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

        // --- Hardware path: HL go_to (cflib HighLevelCommander, COMMAND_GO_TO_2) ---
        const meters = Number(args.METERS);
        if (Math.abs(meters) < 1e-6) {
            return this._runHardwareTimedCommand('copter_fly_distance', util, {
                durationMs: 1,
                start: () => { },
                finish: () => this.init_start_coordinates()
            });
        }

        return this._runHardwareTargetCommand('copter_fly_distance', util, {
            timeoutMs: HARDWARE_LONG_TARGET_COMMAND_TIMEOUT_MS,
            intervalMs: 200,
            prepare: () => {
                this.init_start_coordinates();
                // HL go_to(relative): dx/dy in body frame (cflib MotionCommander convention).
                const {bdx, bdy} = this._hardwareBodyOffsetMeters(meters, this.dir);
                const {dx, dy} = this._hardwareWorldDeltaFromBody(bdx, bdy, this.yaw);
                const tx = this.x + dx;
                const ty = this.y + dy;
                const tz = Number(this.QCA.get_coord('Z'));
                const moveVel = this._hardwareSpeedMps();
                const durationS = Math.max(0.25, Math.abs(meters) / moveVel);
                const yawRad = 0;
                const deadlineMs = Date.now() + durationS * 1000 + HARDWARE_HL_EXTRA_WAIT_MS;
                return {
                    dx: bdx, dy: bdy, tx, ty, tz, yawRad, durationS, deadlineMs,
                    startX: this.x, startY: this.y, startZ: this.z,
                    hlScheduled: false
                };
            },
            dispatch: context => {
                if (context.hlScheduled) return;
                context.hlScheduled = true;
                this.QCA.goToHighLevel(context.dx, context.dy, 0, context.yawRad, context.durationS, { relative: true, forceLegacy: true });
            },
            isDone: context => {
                if (!this.QCA.isQuadcopterConnected()) return true;
                const x = Number(this.QCA.get_coord('X'));
                const y = Number(this.QCA.get_coord('Y'));
                const z = Number(this.QCA.get_coord('Z'));
                if (![x, y, z].every(Number.isFinite)) return false;
                const distanceToTarget = Math.hypot(x - context.tx, y - context.ty, z - context.tz);
                const reachedTarget = distanceToTarget < HARDWARE_POS_TOL_M;
                const deadlineReached = Date.now() >= context.deadlineMs;
                return reachedTarget || deadlineReached;
            },
            finish: () => {
                this.QCA.hoverStop();
                this.init_start_coordinates();
            }
        });
    }

    copter_fly_time(args, util) {
        if (this.fack === 0 && args.DIRECTION !== undefined) {
            const dirKey = this._normalizeDirectionArg(args.DIRECTION);
            if (this._isVerticalFlightDirection(dirKey)) {
                this.flightDirKey = dirKey;
            } else {
                this._applyDirectionKey(dirKey);
            }
        }
        const activeDirKey = this._resolveFlightDirectionFromArgs(args);
        if (this._isVerticalFlightDirection(activeDirKey)) {
            return this._copterFlyTimeVertical(activeDirKey, args.SECONDS, util);
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

        // --- Hardware path ---
        return this._runHardwareTimedCommand('copter_fly_time', util, {
            durationMs: Number(args.SECONDS) * 1000,
            start: () => {
                this.init_start_coordinates();
                this.z = Number(this.QCA.get_coord("Z"));
                const {vx, vy} = this._hardwareBodyVelocity(this._hardwareSpeedMps(), this.dir);
                this.QCA.move_with_speed(vx, vy, 0, this.z);
            },
            finish: () => {
                this.x = this.QCA.get_coord("X");
                this.y = this.QCA.get_coord("Y");
                this.QCA.hoverStop();
                this.init_start_coordinates();
            }
        });
    }

    copter_fly_for_time_with_speed(args, util) {
        if (this.runtime.sim_copter_ac) {
            if (!this._ensureSimCopterSprite(util)) return;
            if (this.fack === 0) {
                this._syncFromSpritePositionIfNeeded();
                if (!this._simCanExecuteAirCommand()) return;
            }
            if (this.fack === 0) {
                const vx = Number(args.X_SPEED);
                const vy = Number(args.Y_SPEED);
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

        // --- Hardware path ---
        return this._runHardwareTimedCommand('copter_fly_for_time_with_speed', util, {
            durationMs: Number(args.SECONDS) * 1000,
            start: () => {
                const vx = Number(args.X_SPEED);
                const vy = Number(args.Y_SPEED);
                this.z = Number(this.QCA.get_coord("Z"));
                this.QCA.move_with_speed(vx, vy, 0, this.z);
            },
            finish: () => {
                this.QCA.hoverStop();
                this.init_start_coordinates();
            }
        });
    }

    copter_change_x_by(args, util) {
        if (this.runtime.sim_copter_ac) {
            if (!this._ensureSimCopterSprite(util)) return;
            if (this.fack === 0) {
                this._syncFromSpritePositionIfNeeded();
                if (!this._simCanExecuteAirCommand()) return;
            }
            if (this.fack === 0) {
                this.sim_x = this.sim_x + Number(args.DISTANCE_DELTA);
                this._sim_target_x = this.sim_x;
                this._simStartMoveToCoord(this.sim_x, this.sim_y, this.sim_z, this.sim_yaw);
                this.yielded_time_start = Date.now();
                this.fack = 1;
                util.yield();
                return;
            } else if (this.fack !== 2) {
                if ((Date.now() - this.yielded_time_start) >= this.yielded_max_time) this.fack = 2;
                if (this._simIsAtTarget(this._sim_target_x, this.sim_y, this.sim_z, this.sim_yaw)) this.fack = 2;
                util.yield();
                return;
            }
            this._simClearInterval();
            this.sim_x = this._sim_target_x;
            this._simApplyState();
            this.fack = 0;
            return;
        }

        // --- Hardware path ---
        this.init_start_coordinates();
        const dx = Number(args.DISTANCE_DELTA);
        const v = this._hardwareSpeedMps();
        const durationMs = Math.abs(dx) < 1e-6
            ? 1
            : Math.max(3000, (Math.abs(dx) / v) * 1000);
        const vx = (dx >= 0 ? 1 : -1) * v;
        return this._runHardwareTimedCommand('copter_change_x_by', util, {
            durationMs,
            start: () => {
                this.z = Number(this.QCA.get_coord("Z"));
                this.QCA.move_with_speed(vx, 0, 0, this.z);
            },
            finish: () => {
                this.QCA.hoverStop();
                this.init_start_coordinates();
            }
        });
    }

    copter_change_y_by(args, util) {
        if (this.runtime.sim_copter_ac) {
            if (!this._ensureSimCopterSprite(util)) return;
            if (this.fack === 0) {
                this._syncFromSpritePositionIfNeeded();
                if (!this._simCanExecuteAirCommand()) return;
            }
            if (this.fack === 0) {
                this.sim_y = this.sim_y - Number(args.DISTANCE_DELTA);
                this._sim_target_y = this.sim_y;
                this._simStartMoveToCoord(this.sim_x, this.sim_y, this.sim_z, this.sim_yaw);
                this.yielded_time_start = Date.now();
                this.fack = 1;
                util.yield();
                return;
            } else if (this.fack !== 2) {
                if ((Date.now() - this.yielded_time_start) >= this.yielded_max_time) this.fack = 2;
                if (this._simIsAtTarget(this.sim_x, this._sim_target_y, this.sim_z, this.sim_yaw)) this.fack = 2;
                util.yield();
                return;
            }
            this._simClearInterval();
            this.sim_y = this._sim_target_y;
            this._simApplyState();
            this.fack = 0;
            return;
        }

        // --- Hardware path ---
        this.init_start_coordinates();
        const dy = -Number(args.DISTANCE_DELTA);
        const v = this._hardwareSpeedMps();
        const durationMs = Math.abs(dy) < 1e-6
            ? 1
            : Math.max(3000, (Math.abs(dy) / v) * 1000);
        const vy = (dy >= 0 ? 1 : -1) * v;
        return this._runHardwareTimedCommand('copter_change_y_by', util, {
            durationMs,
            start: () => {
                this.z = Number(this.QCA.get_coord("Z"));
                this.QCA.move_with_speed(0, vy, 0, this.z);
            },
            finish: () => {
                this.QCA.hoverStop();
                this.init_start_coordinates();
            }
        });
    }

    copter_change_z_by(args, util) {
        if (this.runtime.sim_copter_ac) {
            if (!this._ensureSimCopterSprite(util)) return;
            if (this.fack === 0) {
                this._syncFromSpritePositionIfNeeded();
                if (!this._simCanExecuteAirCommand()) return;
            }
            if (this.fack === 0) {
                this._sim_target_z = Math.max(0, this.sim_z + Number(args.DISTANCE_DELTA));
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

        // --- Hardware path ---
        if (Number(this.QCA.get_coord('Z')) + Number(args.DISTANCE_DELTA) <= GROUND_TARGET_Z_M) {
            return this.copter_land({}, util);
        }
        return this._runHardwareTargetCommand('copter_change_z_by', util, {
            prepare: () => {
                this.init_start_coordinates();
                const startZ = this.z;
                this.z = this.z + Number(args.DISTANCE_DELTA);
                return this._startHeightRamp(startZ);
            },
            dispatch: context => {
                this.QCA.move_with_speed(0, 0, 0, this._stepHeightRamp(context, this.z));
            },
            isDone: () => {
                this.nowz = Number(this.QCA.get_coord("Z"));
                return Math.abs(this.nowz - this.z) < this.delta;
            },
            finish: () => {
                this.QCA.hoverStop();
                this.init_start_coordinates();
            }
        });
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

    copter_fly_for_seconds_to_coords(args, util) {
        if (this.runtime.sim_copter_ac) {
            if (!this._ensureSimCopterSprite(util)) return;
            if (this.fack === 0) {
                this._syncFromSpritePositionIfNeeded();
                if (!this._simCanExecuteAirCommand()) return;
            }
            if (this.fack === 0) {
                const seconds = Number(args.SECONDS);
                const tx = Number(args.X_COORD);
                // Sim uses Scratch-aligned Y (same as sim_y / reporter); do not apply hardware Y flip.
                const ty = Number(args.Y_COORD);
                const tz = Number(args.Z_COORD);
                const vx = seconds > 0 ? (tx - this.sim_x) / seconds : 0;
                const vy = seconds > 0 ? (ty - this.sim_y) / seconds : 0;
                const vz = seconds > 0 ? (tz - this.sim_z) / seconds : 0;
                this._sim_flySecondsEndX = tx;
                this._sim_flySecondsEndY = ty;
                this._sim_flySecondsEndZ = tz;
                this._simClearInterval();
                this.sim_interval = setInterval(() => {
                    this.sim_x += vx * (SIM_STEP_MS / 1000);
                    this.sim_y += vy * (SIM_STEP_MS / 1000);
                    this.sim_z = Math.max(0, this.sim_z + vz * (SIM_STEP_MS / 1000));
                    this._simEnsureFlyingFlagByAltitude();
                    this._simApplyState();
                }, SIM_STEP_MS);
                this.fack = 1;
                this._simAddTimeout(() => {
                    this.fack = 2;
                }, seconds * 1000);
                util.yield();
                return;
            } else if (this.fack !== 2) {
                util.yield();
                return;
            }
            this._simClearInterval();
            this.sim_x = this._sim_flySecondsEndX;
            this.sim_y = this._sim_flySecondsEndY;
            this.sim_z = Math.max(0, this._sim_flySecondsEndZ);
            this._simEnsureFlyingFlagByAltitude();
            this._simApplyState();
            this.fack = 0;
            return;
        }

        // --- Hardware path ---
        return this._runHardwareTimedCommand('copter_fly_for_seconds_to_coords', util, {
            durationMs: Number(args.SECONDS) * 1000,
            start: () => {
                this.x_telemetry_delta = this.QCA.get_x_telemetry_delta();
                this.y_telemetry_delta = this.QCA.get_y_telemetry_delta();
                this.init_start_coordinates();
                const sec = Number(args.SECONDS);
                const targetX = Number(args.X_COORD) + this.x_telemetry_delta;
                const targetY = (Number(args.Y_COORD) * -1) + this.y_telemetry_delta;
                const vx = sec > 0 ? (targetX - this.x) / sec : 0;
                const vy = sec > 0 ? (targetY - this.y) / sec : 0;
                this.z = Number(args.Z_COORD);
                this.QCA.move_with_speed(vx, vy, 0, this.z);
            },
            finish: () => {
                this.QCA.hoverStop();
                this.init_start_coordinates();
            }
        });
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
     * Hardware path: fly to world coordinates without changing heading.
     * Uses HL relative go_to with a world-frame offset and yaw offset 0: the onboard
     * planner adds the offset in world coordinates and keeps its current setpoint yaw
     * (planner.c plan_go_to_from), so coordinates stay exact and the craft does not rotate.
     * @param {number} tx
     * @param {number} ty
     * @param {number} tz
     * @param {object} util
     */
    _hardwareFlyToWorld(tx, ty, targetZ, util) {
        // A target on the ground is a landing: fly over the point low, then land there.
        const landAfter = targetZ <= GROUND_TARGET_Z_M;
        const tz = landAfter ? HARDWARE_PRE_LAND_HEIGHT_M : targetZ;
        return this._runHardwareTargetCommand('copter_fly_to_coords', util, {
            timeoutMs: HARDWARE_LONG_TARGET_COMMAND_TIMEOUT_MS,
            intervalMs: 200,
            prepare: () => {
                this.init_start_coordinates();
                // Firmware relative go_to adds the offset in the WORLD frame
                // (planner.c: hover_pos = vadd(hover_pos, curr_pos)) and keeps the
                // current heading when yaw offset is 0 — so no rotation, exact coords.
                const dx = tx - this.x;
                const dy = ty - this.y;
                const dz = tz - this.z;
                const pathLen = Math.hypot(dx, dy, dz);
                if (pathLen < 1e-6 && !landAfter) {
                    return { skip: true };
                }
                const moveVel = this._hardwareSpeedMps();
                const durationS = Math.max(0.25, pathLen / moveVel);
                const yawRad = 0;
                const deadlineMs = Date.now() + durationS * 1000 + HARDWARE_HL_EXTRA_WAIT_MS;
                return {
                    skip: false,
                    landAfter: landAfter,
                    landing: false,
                    dx: dx,
                    dy: dy,
                    dz: dz,
                    tx: tx,
                    ty: ty,
                    tz: tz,
                    yawRad: yawRad,
                    durationS: durationS,
                    deadlineMs: deadlineMs,
                    hlScheduled: false
                };
            },
            dispatch: context => {
                if (context.skip || context.hlScheduled) {
                    return;
                }
                context.hlScheduled = true;
                this.QCA.goToHighLevel(
                    context.dx, context.dy, context.dz, context.yawRad, context.durationS,
                    {relative: true, forceLegacy: true}
                );
            },
            isDone: context => {
                if (context.skip) {
                    return true;
                }
                if (!this.QCA.isQuadcopterConnected()) {
                    return true;
                }
                const x = Number(this.QCA.get_coord('X'));
                const y = Number(this.QCA.get_coord('Y'));
                const z = Number(this.QCA.get_coord('Z'));
                if (![x, y, z].every(Number.isFinite)) {
                    return false;
                }
                if (context.landing) {
                    return Date.now() >= context.landDeadlineMs;
                }
                const distanceToTarget = Math.hypot(x - context.tx, y - context.ty, z - context.tz);
                const reachedTarget = distanceToTarget < HARDWARE_POS_TOL_M;
                const deadlineReached = Date.now() >= context.deadlineMs;
                if ((reachedTarget || deadlineReached) && context.landAfter) {
                    const landOpts = { velocityMps: this._hardwareSpeedMps() };
                    context.landing = true;
                    context.landDeadlineMs = Date.now() + this.QCA.computeHlLandWaitMs(landOpts);
                    this.QCA.landAndClose(landOpts);
                    return false;
                }
                return reachedTarget || deadlineReached;
            },
            finish: context => {
                if (!context || !context.landing) {
                    this.QCA.hoverStop();
                }
                this.init_start_coordinates();
            }
        });
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
        const magnitude = Math.abs(Number(args.DEGREES));
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

        // --- Hardware path ---
        if (!this._yieldUntilVelocityStreamingReady(util)) {
            return;
        }
        const rotateOwnerId = util.thread && util.thread.topBlock
            ? String(util.thread.topBlock)
            : '';
        const activeRotate = this.commandCoordinator.activeCommand;
        const rotateInProgress = activeRotate &&
            activeRotate.key === 'copter_rotate' &&
            activeRotate.ownerId === rotateOwnerId;
        if (!rotateInProgress) {
            this.init_start_coordinates();
            const turn = signedDeg;
            const yawRateDps = Math.min(
                HARDWARE_MAX_YAW_RATE_DPS,
                HARDWARE_YAW_RATE_DPS * this._hardwareSpeedMps() / DEFAULT_SPEED_MPS
            );
            const durationMs = (magnitude / yawRateDps) * 1000;
            const yawRate = magnitude < 1e-3 ? 0 : Math.sign(turn) * yawRateDps;
            this._hwRotatePlan = {
                ownerId: rotateOwnerId,
                side,
                signedDeg,
                turn,
                durationMs,
                yawRate
            };
        }
        const plan = this._hwRotatePlan;
        if (!plan) return;
        return this._runHardwareTimedCommand('copter_rotate', util, {
            durationMs: plan.durationMs,
            start: () => {
                this.z = Number(this.QCA.get_coord('Z'));
                this.QCA.move_with_speed(0, 0, plan.yawRate, this.z);
                return plan;
            },
            finish: () => {
                this.QCA.hoverStop();
                this.init_start_coordinates();
                this._hwRotatePlan = null;
            },
            cancel: () => {
                this._hwRotatePlan = null;
            }
        });
    }

    copter_set_direction(args) {
        const raw = args.DIRECTION !== undefined ? args.DIRECTION : args.COPTER_DIRECTIONS;
        this._applyDirectionKey(this._normalizeDirectionArg(raw));
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

    _applyDirectionKey(key) {
        this.flightDirKey = key;
        this.dir = COPTER_DIR_DEGREES[key] !== undefined ? COPTER_DIR_DEGREES[key] : 0;
    }

    _dirKeyFromDegrees(degrees) {
        const d = Number(degrees);
        if (d === 90) return 'left';
        if (d === 180) return 'backward';
        if (d === 270) return 'right';
        return 'forward';
    }

    _dirLabelFromKey(key) {
        const resolvedKey = COPTER_DIR_MSG[key] ? key : 'forward';
        const msgDef = COPTER_DIR_MSG[resolvedKey];
        const viaFormat = formatMessage(msgDef);
        if (viaFormat !== msgDef.default) {
            return viaFormat;
        }
        const locale = (this.runtime && this.runtime.getLocale) ? this.runtime.getLocale() : 'en';
        const table = COPTER_DIR_LABELS_BY_LOCALE[locale] || COPTER_DIR_LABELS_BY_LOCALE.en;
        return table[key] || table.forward;
    }

    copter_direction() {
        const key = this.flightDirKey && COPTER_DIR_MSG[this.flightDirKey]
            ? this.flightDirKey
            : this._dirKeyFromDegrees(this.dir);
        return this._dirLabelFromKey(key);
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

    copter_reset_local_position() {
        if (this.runtime.sim_copter_ac) {
            if (this.sim_is_flying || this.sim_z > 0.02) {
                return;
            }
            // The current place becomes the start point; the sprite does not move.
            this.sim_origin_x += this.sim_x;
            this.sim_origin_y += this.sim_y;
            this.sim_x = 0;
            this.sim_y = 0;
            this._simApplyState();
            return;
        }

        if (!this.QCA ||
            typeof this.QCA.canResetLocalPosition !== 'function' ||
            typeof this.QCA.resetLocalPosition !== 'function') {
            return;
        }

        if (this.QCA.canResetLocalPosition()) {
            this.QCA.resetLocalPosition();
        }
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

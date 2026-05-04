const Cast = require('../util/cast');
const MathUtil = require('../util/math-util');
const Timer = require('../util/timer');
const QuadcopterCommandCoordinator = require('./quadcopter-command-coordinator');

const COPTER_SPRITE_NAME = 'Robbo Quadcopter';
const COSTUME_IDLE = 0;
const COSTUME_FLYING = 1;
const SIM_SCALE = 200;       // 1 meter = 200 Scratch pixels
const SIM_BASE_SIZE = 15;    // sprite size (%) at z=0
const SIM_SIZE_PER_METER = 12; // +12% per meter of altitude
const SIM_STEP_MS = 50;      // physics tick interval
/** Horizontal / default move interpolation (m/s); matches sim `fly time` distance rate. */
const SIM_MOVE_SPEED = 0.4;
/** Vertical ascent rate for takeoff (m/s), faster than {@link SIM_LAND_Z_MPS}. */
const SIM_TAKEOFF_Z_MPS = 0.55;
/** Descent rate while landing (m/s). */
const SIM_LAND_Z_MPS = 0.22;
const SIM_POS_TOLERANCE = 0.004;
const SIM_YAW_TOLERANCE = 2.5;

/**
 * Полёт по железу: в воздухе используем velocity setpoints + hoverStop (как cflib MotionCommander).
 * Исключение: copter_land на железе — один вызов HL land в сессии (как cfclient).
 * @see https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/commanders_setpoints/
 */
const HARDWARE_YAW_RATE_DPS = 72;
/** Ожидание isDone по телеметрии — верхняя граница (секунды полёта редко дольше). */
const HARDWARE_TARGET_COMMAND_TIMEOUT_MS = 45000;
const HARDWARE_LONG_TARGET_COMMAND_TIMEOUT_MS = 120000;
/** HL go_to / distance: finish early when within this 3D error (m). */
const HARDWARE_POS_TOL_M = 0.07;
/** Upper bound on block wait beyond nominal HL trajectory (radio + planner). */
const HARDWARE_HL_EXTRA_WAIT_MS = 650;

class Scratch3QuadcopterBlocks {
    constructor (runtime) {
        /**
         * The runtime instantiating this block package.
         * @type {Runtime}
         */
        this.runtime = runtime;
          this.x = this.runtime.QCA.get_coord("X");
          this.nowx=0;
          this.y =this.runtime.QCA.get_coord("Y");
          this.nowy=0;
          this.z = this.runtime.QCA.get_coord("Z");
          this.nowz=0;
          this.defz= 0.3;
          this.yaw =this.runtime.QCA.get_coord("W");
          this.noww=0;
          this.dir = 0;
          this.fack = 0;
          this.delta= 0.01;
          this.speed= 1;

          this.yielded_time_start = Date.now();
          this.yielded_time_now   = Date.now();
          this.yielded_max_time = 1 * 60 * 1000;

          this.x_telemetry_delta = 0;
          this.y_telemetry_delta = 0;

          // --- Simulator state ---
          this.runtime.sim_copter_ac = false;
          this.runtime._copterSimBlocks = this;
          this.sim_x = 0;
          this.sim_y = 0;
          this.sim_z = 0;
          this.sim_yaw = 0;
          this.sim_is_flying = false;
          this.sim_battery = 100;
          this.sim_interval = null;
          this.sim_base_size = SIM_BASE_SIZE;

          this.SendCordInterval = null;
          this.CopterLANDING = null;
          this._simTimeouts = new Set();
          this.commandCoordinator = new QuadcopterCommandCoordinator();

          this.runtime.on('PROJECT_STOP_ALL', this._onProjectStopAll.bind(this));
          this.runtime.on('ROBBO_SIM_SPRITES_INVALIDATED', this._onRobboSimSpritesInvalidated.bind(this));
    }

    getPrimitives () {
        return {
            copter_fly_up: this.copter_fly_up,
            copter_land:this.copter_land,
            copter_stop:this.copter_stop,
            copter_status:this.copter_status,
            copter_fly_distance:this.copter_fly_distance,
            copter_fly_time:this.copter_fly_time,
            copter_fly_for_time_with_speed:this.copter_fly_for_time_with_speed,
            copter_change_x_by:this.copter_change_x_by,
            copter_change_y_by:this.copter_change_y_by,
            copter_change_z_by:this.copter_change_z_by,
            copter_x_coord:this.copter_x_coord,
            copter_y_coord:this.copter_y_coord,
            copter_z_coord:this.copter_z_coord,
            copter_yaw:this.copter_yaw,
            copter_fly_for_seconds_to_coords:this.copter_fly_for_seconds_to_coords,
            copter_fly_to_coords:this.copter_fly_to_coords,
            copter_rotate:this.copter_rotate,
            copter_set_direction:this.copter_set_direction,
            copter_direction:this.copter_direction
        };
    }

    getMonitored () {
        return {};
    }

    // ===== Simulator helpers =====

    _getSimCopterTarget () {
        if (!this.runtime || !this.runtime.targets) return null;
        for (let i = 0; i < this.runtime.targets.length; i++) {
            const t = this.runtime.targets[i];
            if (t.isOriginal && !t.isStage && t.sprite && t.sprite.name === COPTER_SPRITE_NAME) {
                return t;
            }
        }
        return null;
    }

    _simApplyState () {
        const target = this._getSimCopterTarget();
        if (!target) {
            if (this.runtime && this.runtime.sim_copter_ac) {
                this.runtime.sim_copter_ac = false;
                this._simClearInterval();
                this._simClearAllTimeouts();
                this.fack = 0;
            }
            return;
        }
        if (!target.draggable) {
            target.setDraggable(true);
        }
        target.setXY(this.sim_x * SIM_SCALE, this.sim_y * SIM_SCALE);
        target.setDirection(this.sim_yaw);
        const sz = this.sim_base_size + this.sim_z * SIM_SIZE_PER_METER;
        target.setSize(sz);
        const costumeIdx = this.sim_is_flying ? COSTUME_FLYING : COSTUME_IDLE;
        if (target.currentCostume !== costumeIdx) {
            target.setCostume(costumeIdx);
        }
    }

    _simCanExecuteAirCommand () {
        return this.sim_is_flying === true && this.sim_z > 0.02;
    }

    _simEnsureFlyingFlagByAltitude () {
        if (this.sim_z <= 0.02) {
            this.sim_z = 0;
            this.sim_is_flying = false;
            return;
        }
        this.sim_is_flying = true;
    }

    syncFromSpritePosition () {
        if (!this.runtime || !this.runtime.sim_copter_ac) return false;
        const target = this._getSimCopterTarget();
        if (!target) {
            this.runtime.sim_copter_ac = false;
            this._simClearInterval();
            this._simClearAllTimeouts();
            this.fack = 0;
            return false;
        }
        let changed = false;
        const nextX = Number((target.x / SIM_SCALE).toFixed(3));
        const nextY = Number((target.y / SIM_SCALE).toFixed(3));
        if (Math.abs(nextX - this.sim_x) > 0.0005 || Math.abs(nextY - this.sim_y) > 0.0005) {
            this.sim_x = nextX;
            this.sim_y = nextY;
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

    _isSimMotionActive () {
        return !!this.sim_interval;
    }

    _shouldSyncFromSpritePosition () {
        if (!this.runtime || !this.runtime.sim_copter_ac) return false;
        const target = this._getSimCopterTarget();
        if (!target) return false;
        if (this._isSimMotionActive()) return true;
        const nextX = Number((target.x / SIM_SCALE).toFixed(3));
        const nextY = Number((target.y / SIM_SCALE).toFixed(3));
        if (Math.abs(nextX - this.sim_x) > 0.0005 || Math.abs(nextY - this.sim_y) > 0.0005) {
            return true;
        }
        const dir = Number(target.direction);
        if (!Number.isFinite(dir)) return false;
        const nextYaw = this._castYawTo360(dir);
        let dyaw = Math.abs(nextYaw - this.sim_yaw);
        if (dyaw > 180) dyaw = 360 - dyaw;
        return dyaw > 0.05;
    }

    _syncFromSpritePositionIfNeeded () {
        if (!this._shouldSyncFromSpritePosition()) return false;
        return this.syncFromSpritePosition();
    }

    setStateFromPaletteInput (nextState) {
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

    _simClearInterval () {
        if (this.sim_interval) {
            clearInterval(this.sim_interval);
            this.sim_interval = null;
        }
    }

    _simAddTimeout (fn, ms) {
        const id = setTimeout(() => {
            this._simTimeouts.delete(id);
            fn();
        }, ms);
        this._simTimeouts.add(id);
        return id;
    }

    _simClearAllTimeouts () {
        for (const id of this._simTimeouts) {
            clearTimeout(id);
        }
        this._simTimeouts.clear();
    }

    _onProjectStopAll () {
        this._simClearInterval();
        this._simClearAllTimeouts();
        this.commandCoordinator.cancel('projectStopAll');
        this._clearHardwareCommandIntervals();
        this.fack = 0;
    }

    /**
     * When the copter sim sprite is removed while sim was active, clear sim timers.
     * @param {Object} payload
     * @param {boolean} [payload.robot]
     * @param {boolean} [payload.copter]
     */
    _onRobboSimSpritesInvalidated (payload) {
        if (payload && payload.copter) {
            this._onProjectStopAll();
        }
    }

    /**
     * @param {object} util
     * @returns {boolean} true when copter sprite exists (or sim off)
     */
    _ensureSimCopterSprite (util) {
        if (!this.runtime.sim_copter_ac) return true;
        if (this._getSimCopterTarget()) return true;
        this.runtime.sim_copter_ac = false;
        this._simClearInterval();
        this._simClearAllTimeouts();
        this.fack = 0;
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
    _simStartMoveToCoord (tx, ty, tz, tyaw, opts) {
        this._simClearInterval();
        const xyMps = opts && Number.isFinite(opts.xyMps) ? opts.xyMps : SIM_MOVE_SPEED;
        const zMps = opts && Number.isFinite(opts.zMps) ? opts.zMps : SIM_MOVE_SPEED;
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
            const yawStep = 90 * (SIM_STEP_MS / 1000); // 90 deg/s
            if (Math.abs(dyaw) > yawStep) {
                this.sim_yaw += Math.sign(dyaw) * yawStep;
            } else {
                this.sim_yaw = tyaw;
            }
            this.sim_yaw = this._castYawTo360(this.sim_yaw);
            this._simApplyState();
        }, SIM_STEP_MS);
    }

    _simIsAtTarget (tx, ty, tz, tyaw) {
        let dyaw = Math.abs(tyaw - this.sim_yaw);
        if (dyaw > 180) dyaw = 360 - dyaw;
        return Math.abs(this.sim_x - tx) < SIM_POS_TOLERANCE &&
               Math.abs(this.sim_y - ty) < SIM_POS_TOLERANCE &&
               Math.abs(this.sim_z - tz) < SIM_POS_TOLERANCE &&
               dyaw < SIM_YAW_TOLERANCE;
    }

    _castYawTo360 (yaw) {
        if (yaw > 0) {
            yaw = yaw - ((yaw / 360) >> 0) * 360;
        } else if (yaw < 0) {
            yaw = yaw + ((yaw / 360) >> 0) * -360;
        }
        return yaw;
    }

    _minHardwareSpeed () {
        return Math.max(Math.abs(Number(this.speed)) || 0, 0.12);
    }

    _shortestYawDeltaDeg (fromDeg, toDeg) {
        const from = Number(fromDeg);
        const to = Number(toDeg);
        if (!Number.isFinite(from) || !Number.isFinite(to)) return 0;
        let d = (to - from) % 360;
        if (d > 180) d -= 360;
        if (d < -180) d += 360;
        return d;
    }

    _clearHardwareCommandIntervals () {
        if (this.SendCordInterval) {
            clearInterval(this.SendCordInterval);
            this.SendCordInterval = null;
        }
        if (this.CopterLANDING) {
            clearInterval(this.CopterLANDING);
            this.CopterLANDING = null;
        }
    }

    _runHardwareTargetCommand (commandKey, util, opts) {
        const ceiling = opts.timeoutMs !== undefined
            ? opts.timeoutMs
            : HARDWARE_TARGET_COMMAND_TIMEOUT_MS;
        this.commandCoordinator.runTargetCommand(commandKey, util, {
            timeoutMs: ceiling,
            start: () => {
                if (typeof opts.prepare === 'function') {
                    opts.prepare();
                }
                const context = {
                    disconnected: false
                };
                context.intervalId = setInterval(() => {
                    if (!this.runtime.QCA.isQuadcopterConnected()) {
                        context.disconnected = true;
                        return;
                    }
                    opts.dispatch();
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
                    this.runtime.QCA.hoverStop();
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

    _runHardwareTimedCommand (commandKey, util, opts) {
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
                if (!this.runtime.QCA.isQuadcopterConnected()) {
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

    copter_fly_up (args, util) {
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
                this._sim_target_z = this.sim_z + 0.3;
                this.sim_is_flying = true;
                this._simStartMoveToCoord(this.sim_x, this.sim_y, this._sim_target_z, this.sim_yaw, {
                    zMps: SIM_TAKEOFF_Z_MPS
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
            this._simApplyState();
            this.fack = 0;
            return;
        }

        // --- Hardware path: HL takeoff once (mirrors cfclient Flight Tab Take off button) ---
        return this._runHardwareTargetCommand('copter_fly_up', util, {
            timeoutMs: HARDWARE_LONG_TARGET_COMMAND_TIMEOUT_MS,
            intervalMs: 200,
            prepare: () => {
                this.init_start_coordinates();
                this.z = this.z + 0.3;
                this._copterTakeoffHlScheduled = false;
            },
            dispatch: () => {
                if (this._copterTakeoffHlScheduled) {
                    return;
                }
                this._copterTakeoffHlScheduled = true;
                this.runtime.QCA.takeoff(this.z);
            },
            isDone: () => {
                if (!this.runtime.QCA.isQuadcopterConnected()) {
                    return true;
                }
                const nowZ = Number(this.runtime.QCA.get_coord("Z"));
                // HL takeoff may use a higher absolute target than `this.z` when telemetry Z is stale
                // (see QCA.takeoff); finish once we've reached at least the block's nominal height.
                return Number.isFinite(nowZ) && nowZ >= this.z - this.delta;
            },
            finish: () => {
                this.init_start_coordinates();
            }
        });
    }

    copter_land (args, util) {
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
                    this.sim_z -= SIM_LAND_Z_MPS * (SIM_STEP_MS / 1000);
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
                this._copterLandHlScheduled = false;
                const estimateMs = typeof this.runtime.QCA.computeHlLandWaitMs === 'function'
                    ? this.runtime.QCA.computeHlLandWaitMs()
                    : 4000;
                this._landBlockDeadlineMs = Date.now() + estimateMs;
            },
            dispatch: () => {
                if (this._copterLandHlScheduled) {
                    return;
                }
                this._copterLandHlScheduled = true;
                this.runtime.QCA.landAndClose();
            },
            isDone: (context, elapsedMs) => {
                if (!this.runtime.QCA.isQuadcopterConnected()) {
                    return true;
                }
                if (typeof this._landBlockDeadlineMs === 'number') {
                    return Date.now() >= this._landBlockDeadlineMs;
                }
                return elapsedMs >= 4000;
            },
            finish: () => {
                this.init_start_coordinates();
            }
        });
    }

    copter_stop () {
        if (this.runtime.sim_copter_ac) {
            this._simClearInterval();
            this._simClearAllTimeouts();
            this.sim_is_flying = false;
            this.sim_z = 0;
            this._simApplyState();
            this.fack = 0;
            return;
        }

        // --- Hardware path ---
        this.commandCoordinator.cancel('manualStop');
        this._clearHardwareCommandIntervals();
        this.runtime.QCA.landAndClose();
    }

    copter_status (args, util) {
        if (this.runtime.sim_copter_ac) {
            if (!this._getSimCopterTarget()) {
                this.runtime.sim_copter_ac = false;
                return false;
            }
            return true;
        }
        return(this.runtime.QCA.isQuadcopterConnected());
    }

    copter_fly_distance (args, util) {
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
        this.init_start_coordinates();
        const meters = Number(args.METERS);
        if (Math.abs(meters) < 1e-6) {
            return this._runHardwareTimedCommand('copter_fly_distance', util, {
                durationMs: 1,
                start: () => {},
                finish: () => this.init_start_coordinates()
            });
        }
        const heading = (this.yaw + this.dir) * Math.PI / 180;
        const tx = this.x + meters * Math.cos(heading);
        const ty = this.y + meters * Math.sin(heading);
        const tz = Number(this.runtime.QCA.get_coord('Z'));
        const moveVel = typeof this.runtime.QCA.getFlightMoveVelocityMps === 'function'
            ? this.runtime.QCA.getFlightMoveVelocityMps()
            : 0.5;
        const durationS = Math.max(0.25, Math.abs(meters) / moveVel);
        const yawRad = typeof this.runtime.QCA.telemetryYawToHeadingRad === 'function'
            ? this.runtime.QCA.telemetryYawToHeadingRad(Number(this.runtime.QCA.get_coord('W')))
            : Number(this.runtime.QCA.get_coord('W'));
        const deadlineMs = Date.now() + durationS * 1000 + HARDWARE_HL_EXTRA_WAIT_MS;

        return this._runHardwareTargetCommand('copter_fly_distance', util, {
            timeoutMs: HARDWARE_LONG_TARGET_COMMAND_TIMEOUT_MS,
            intervalMs: 200,
            prepare: () => {
                this._copterGoToDistScheduled = false;
            },
            dispatch: () => {
                if (this._copterGoToDistScheduled) return;
                this._copterGoToDistScheduled = true;
                this.runtime.QCA.goToHighLevel(tx, ty, tz, yawRad, durationS);
            },
            isDone: () => {
                if (!this.runtime.QCA.isQuadcopterConnected()) return true;
                if (Date.now() >= deadlineMs) return true;
                const x = Number(this.runtime.QCA.get_coord('X'));
                const y = Number(this.runtime.QCA.get_coord('Y'));
                const z = Number(this.runtime.QCA.get_coord('Z'));
                if (![x, y, z].every(Number.isFinite)) return false;
                return Math.hypot(x - tx, y - ty, z - tz) < HARDWARE_POS_TOL_M;
            },
            finish: () => {
                this.runtime.QCA.hoverStop();
                this.init_start_coordinates();
            }
        });
    }

    copter_fly_time (args, util) {
        if (this.runtime.sim_copter_ac) {
            if (!this._ensureSimCopterSprite(util)) return;
            if (this.fack === 0) {
                this._syncFromSpritePositionIfNeeded();
                if (!this._simCanExecuteAirCommand()) return;
            }
            if (this.fack === 0) {
                const headingRad = (90 - (this.sim_yaw + this.dir)) * Math.PI / 180;
                const vx = SIM_MOVE_SPEED * Math.cos(headingRad);
                const vy = SIM_MOVE_SPEED * Math.sin(headingRad);
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
                this.z = Number(this.runtime.QCA.get_coord("Z"));
                const vx = this.speed * Math.cos((this.yaw + this.dir) * Math.PI / 180);
                const vy = this.speed * Math.sin((this.yaw + this.dir) * Math.PI / 180);
                this.runtime.QCA.move_with_speed(vx, vy, 0, this.z);
            },
            finish: () => {
                this.x = this.runtime.QCA.get_coord("X");
                this.y = this.runtime.QCA.get_coord("Y");
                this.runtime.QCA.hoverStop();
                this.init_start_coordinates();
            }
        });
    }

    copter_fly_for_time_with_speed (args, util) {
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
                this.z = Number(this.runtime.QCA.get_coord("Z"));
                this.runtime.QCA.move_with_speed(vx, vy, 0, this.z);
            },
            finish: () => {
                this.runtime.QCA.hoverStop();
                this.init_start_coordinates();
            }
        });
    }

    copter_change_x_by (args, util) {
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
        const v = this._minHardwareSpeed();
        const durationMs = Math.abs(dx) < 1e-6
            ? 1
            : Math.max(3000, (Math.abs(dx) / v) * 1000);
        const vx = (dx >= 0 ? 1 : -1) * v;
        return this._runHardwareTimedCommand('copter_change_x_by', util, {
            durationMs,
            start: () => {
                this.z = Number(this.runtime.QCA.get_coord("Z"));
                this.runtime.QCA.move_with_speed(vx, 0, 0, this.z);
            },
            finish: () => {
                this.runtime.QCA.hoverStop();
                this.init_start_coordinates();
            }
        });
    }

    copter_change_y_by (args, util) {
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
        const v = this._minHardwareSpeed();
        const durationMs = Math.abs(dy) < 1e-6
            ? 1
            : Math.max(3000, (Math.abs(dy) / v) * 1000);
        const vy = (dy >= 0 ? 1 : -1) * v;
        return this._runHardwareTimedCommand('copter_change_y_by', util, {
            durationMs,
            start: () => {
                this.z = Number(this.runtime.QCA.get_coord("Z"));
                this.runtime.QCA.move_with_speed(0, vy, 0, this.z);
            },
            finish: () => {
                this.runtime.QCA.hoverStop();
                this.init_start_coordinates();
            }
        });
    }

    copter_change_z_by (args, util) {
        if (this.runtime.sim_copter_ac) {
            if (!this._ensureSimCopterSprite(util)) return;
            if (this.fack === 0) {
                this._syncFromSpritePositionIfNeeded();
                if (!this._simCanExecuteAirCommand()) return;
            }
            if (this.fack === 0) {
                this.sim_z = this.sim_z + Number(args.DISTANCE_DELTA);
                if (this.sim_z < 0) this.sim_z = 0;
                this._sim_target_z = this.sim_z;
                this._simStartMoveToCoord(this.sim_x, this.sim_y, this.sim_z, this.sim_yaw);
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
            this._simApplyState();
            this.fack = 0;
            return;
        }

        // --- Hardware path ---
        return this._runHardwareTargetCommand('copter_change_z_by', util, {
            prepare: () => {
                this.init_start_coordinates();
                this.z = this.z + Number(args.DISTANCE_DELTA);
            },
            dispatch: () => {
                this.runtime.QCA.move_with_speed(0, 0, 0, this.z);
            },
            isDone: () => {
                this.nowz = Number(this.runtime.QCA.get_coord("Z"));
                return Math.abs(this.nowz - this.z) < this.delta;
            },
            finish: () => {
                this.runtime.QCA.hoverStop();
                this.init_start_coordinates();
            }
        });
    }

    copter_x_coord (args, util) {
        if (this.runtime.sim_copter_ac) {
            this._syncFromSpritePositionIfNeeded();
            return Number(this.sim_x.toFixed(3));
        }
        return Number(this.runtime.QCA.telemetry_palette_get_coord("X"));
    }

    copter_y_coord (args, util) {
        if (this.runtime.sim_copter_ac) {
            this._syncFromSpritePositionIfNeeded();
            return Number(this.sim_y.toFixed(3));
        }
        return Number(this.runtime.QCA.telemetry_palette_get_coord("Y"));
    }

    copter_yaw (args, util) {
        if (this.runtime.sim_copter_ac) return Number(this.sim_yaw.toFixed(1));
        return Number(this.runtime.QCA.telemetry_palette_get_coord("W"));
    }

    copter_z_coord (args, util) {
        if (this.runtime.sim_copter_ac) return Number(this.sim_z.toFixed(3));
        return Number(this.runtime.QCA.telemetry_palette_get_coord("Z"));
    }

    copter_fly_for_seconds_to_coords (args, util) {
        if (this.runtime.sim_copter_ac) {
            if (!this._ensureSimCopterSprite(util)) return;
            if (this.fack === 0) {
                this._syncFromSpritePositionIfNeeded();
                if (!this._simCanExecuteAirCommand()) return;
            }
            if (this.fack === 0) {
                const seconds = Number(args.SECONDS);
                const tx = Number(args.X_COORD);
                const ty = Number(args.Y_COORD) * -1;
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
                    this.sim_z += vz * (SIM_STEP_MS / 1000);
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
            this.sim_z = this._sim_flySecondsEndZ;
            this._simApplyState();
            this.fack = 0;
            return;
        }

        // --- Hardware path ---
        return this._runHardwareTimedCommand('copter_fly_for_seconds_to_coords', util, {
            durationMs: Number(args.SECONDS) * 1000,
            start: () => {
                this.init_start_coordinates();
                const sec = Number(args.SECONDS);
                const vx = sec > 0 ? (Number(args.X_COORD) - this.x) / sec : 0;
                const vy = sec > 0 ? (Number(args.Y_COORD) - this.y) / sec : 0;
                this.z = Number(args.Z_COORD);
                this.runtime.QCA.move_with_speed(vx, vy, 0, this.z);
            },
            finish: () => {
                this.runtime.QCA.hoverStop();
                this.init_start_coordinates();
            }
        });
    }

    copter_fly_to_coords (args, util) {
        if (this.runtime.sim_copter_ac) {
            if (!this._ensureSimCopterSprite(util)) return;
            if (this.fack === 0) {
                this._syncFromSpritePositionIfNeeded();
                if (!this._simCanExecuteAirCommand()) return;
            }
            if (this.fack === 0) {
                this._sim_target_x = Number(args.X_COORD);
                this._sim_target_y = Number(args.Y_COORD) * -1;
                this._sim_target_z = Math.max(0, Number(args.Z_COORD));
                this._simStartMoveToCoord(this._sim_target_x, this._sim_target_y, this._sim_target_z, this.sim_yaw);
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
            this._simApplyState();
            this.fack = 0;
            return;
        }

        // --- Hardware path: HL go_to (cflib); 3D path + early stop on telemetry ---
        this.x_telemetry_delta = this.runtime.QCA.get_x_telemetry_delta();
        this.y_telemetry_delta = this.runtime.QCA.get_y_telemetry_delta();
        this.init_start_coordinates();
        const tx = Number(args.X_COORD) + this.x_telemetry_delta;
        const ty = (Number(args.Y_COORD) * -1) + this.y_telemetry_delta;
        const tz = Number(args.Z_COORD);
        const dx = tx - this.x;
        const dy = ty - this.y;
        const dz = tz - this.z;
        const pathLen = Math.sqrt(dx * dx + dy * dy + dz * dz);
        const moveVel = typeof this.runtime.QCA.getFlightMoveVelocityMps === 'function'
            ? this.runtime.QCA.getFlightMoveVelocityMps()
            : 0.5;
        const yawRad = typeof this.runtime.QCA.telemetryYawToHeadingRad === 'function'
            ? this.runtime.QCA.telemetryYawToHeadingRad(Number(this.runtime.QCA.get_coord('W')))
            : Number(this.runtime.QCA.get_coord('W'));

        if (pathLen < 1e-6) {
            return this._runHardwareTimedCommand('copter_fly_to_coords', util, {
                durationMs: 1,
                start: () => {},
                finish: () => this.init_start_coordinates()
            });
        }

        const durationS = Math.max(0.25, pathLen / moveVel);
        const deadlineMs = Date.now() + durationS * 1000 + HARDWARE_HL_EXTRA_WAIT_MS;

        return this._runHardwareTargetCommand('copter_fly_to_coords', util, {
            timeoutMs: HARDWARE_LONG_TARGET_COMMAND_TIMEOUT_MS,
            intervalMs: 200,
            prepare: () => {
                this._copterGoToCoordsScheduled = false;
            },
            dispatch: () => {
                if (this._copterGoToCoordsScheduled) return;
                this._copterGoToCoordsScheduled = true;
                this.runtime.QCA.goToHighLevel(tx, ty, tz, yawRad, durationS);
            },
            isDone: () => {
                if (!this.runtime.QCA.isQuadcopterConnected()) return true;
                if (Date.now() >= deadlineMs) return true;
                const x = Number(this.runtime.QCA.get_coord('X'));
                const y = Number(this.runtime.QCA.get_coord('Y'));
                const z = Number(this.runtime.QCA.get_coord('Z'));
                if (![x, y, z].every(Number.isFinite)) return false;
                return Math.hypot(x - tx, y - ty, z - tz) < HARDWARE_POS_TOL_M;
            },
            finish: () => {
                this.runtime.QCA.hoverStop();
                this.init_start_coordinates();
            }
        });
    }

    cast_yaw_to_360 (yaw) {
        return this._castYawTo360(yaw);
    }

    copter_rotate (args, util) {
        if (this.runtime.sim_copter_ac) {
            if (!this._ensureSimCopterSprite(util)) return;
            if (this.fack === 0) {
                this._syncFromSpritePositionIfNeeded();
                if (!this._simCanExecuteAirCommand()) return;
            }
            if (this.fack === 0) {
                this._sim_target_yaw = this._castYawTo360(this.sim_yaw + Number(args.DEGREES));
                this._simStartMoveToCoord(this.sim_x, this.sim_y, this.sim_z, this._sim_target_yaw);
                this.yielded_time_start = Date.now();
                this.fack = 1;
                util.yield();
                return;
            } else if (this.fack !== 2) {
                if ((Date.now() - this.yielded_time_start) >= this.yielded_max_time) this.fack = 2;
                if (this._simIsAtTarget(this.sim_x, this.sim_y, this.sim_z, this._sim_target_yaw)) this.fack = 2;
                util.yield();
                return;
            }
            this._simClearInterval();
            this.sim_yaw = this._sim_target_yaw;
            this._simApplyState();
            this.fack = 0;
            return;
        }

        // --- Hardware path ---
        this.init_start_coordinates();
        const deltaDeg = Number(args.DEGREES);
        const startW = this.yaw;
        const targetW = this._castYawTo360(startW + deltaDeg);
        const turn = this._shortestYawDeltaDeg(startW, targetW);
        const durationMs = Math.max(2500, (Math.abs(turn) / HARDWARE_YAW_RATE_DPS) * 1000);
        const yawRate = Math.abs(turn) < 1e-3 ? 0 : Math.sign(turn) * HARDWARE_YAW_RATE_DPS;
        return this._runHardwareTimedCommand('copter_rotate', util, {
            durationMs,
            start: () => {
                this.z = Number(this.runtime.QCA.get_coord("Z"));
                this.runtime.QCA.move_with_speed(0, 0, yawRate, this.z);
            },
            finish: () => {
                this.runtime.QCA.hoverStop();
                this.init_start_coordinates();
            }
        });
    }

    copter_set_direction (args, util) {
        switch (args.DIRECTION) {
          case 'direction_forward':
            this.dir = 0;
          break;
          case 'direction_right':
            this.dir = 270;
          break;
          case 'direction_backward':
            this.dir = 180;
          break;
          case 'direction_left':
            this.dir = 90;
          break;
        }
    }

    copter_direction (args, util) {
        if (this.runtime.sim_copter_ac) return Number(this.sim_yaw.toFixed(1));
        return this.runtime.QCA.get_coord("W");
    }

    init_start_coordinates () {
        this.yaw = Number(this.runtime.QCA.get_coord("W"));
        this.x = Number(this.runtime.QCA.get_coord("X"));
        this.y = Number(this.runtime.QCA.get_coord("Y"));
        this.z = Number(this.runtime.QCA.get_coord("Z"));
    }
}

module.exports = Scratch3QuadcopterBlocks;

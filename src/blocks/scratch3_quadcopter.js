const Cast = require('../util/cast');
const CopterController = require('./copter-controller');
const {listSimCopterNumbers, MAX_SIM_COPTER_NUMBER} = require('../util/copter-sim-sprite');

/** Copter id of a factory (not yet numbered) Crazyflie in QCA. */
const FACTORY_COPTER_ID = 'new';

/** Copter blocks that act on (or read) the selected copter. */
const COPTER_COMMAND_OPCODES = [
    'copter_fly_up',
    'copter_land',
    'copter_stop',
    'copter_fly_distance',
    'copter_fly_time',
    'copter_fly_for_time_with_speed',
    'copter_change_x_by',
    'copter_change_y_by',
    'copter_change_z_by',
    'copter_move_axis_to',
    'copter_fly_for_seconds_to_coords',
    'copter_fly_to_coords',
    'copter_rotate',
    'copter_set_direction',
    'copter_set_speed',
    'copter_reset_local_position',
    'copter_charge_battery'
];

/** Flight blocks refused by an empty simulator battery (land / stop / settings still work). */
const SIM_FLIGHT_OPCODES = new Set([
    'copter_fly_up',
    'copter_fly_distance',
    'copter_fly_time',
    'copter_fly_for_time_with_speed',
    'copter_change_x_by',
    'copter_change_y_by',
    'copter_change_z_by',
    'copter_move_axis_to',
    'copter_fly_for_seconds_to_coords',
    'copter_fly_to_coords',
    'copter_rotate'
]);

/** Reporters and their value when no copter can be chosen. */
const COPTER_REPORTER_DEFAULTS = {
    copter_status: false,
    copter_x_coord: 0,
    copter_y_coord: 0,
    copter_z_coord: 0,
    copter_yaw: 0,
    copter_direction: 0,
    copter_battery: 0,
    copter_is_flying: false,
    copter_speed: 0
};

/** `[X ▾] of copter [N]` properties → controller reporter. */
const COPTER_PROPERTY_REPORTERS = {
    x: 'copter_x_coord',
    y: 'copter_y_coord',
    z: 'copter_z_coord',
    yaw: 'copter_yaw',
    battery: 'copter_battery',
    flying: 'copter_is_flying',
    link: 'copter_link_quality'
};

/** Menu value for every enabled copter. */
const ALL_COPTERS = 'all';

/**
 * Quadcopter block package: routes every copter block to the {@link CopterController}s selected by
 * the enclosing `copters [N] do` C-block (a per-thread selection stack: a number, a set such as
 * "1, 3" / "1-3", or "all"). Without a selection blocks drive every active copter. A command for
 * several copters starts on each of them and finishes when all are done.
 */
class Scratch3QuadcopterBlocks {
    constructor (runtime) {
        /**
         * The runtime instantiating this block package.
         * @type {Runtime}
         */
        this.runtime = runtime;
        this.runtime.sim_copter_ac = false;
        /** @type {Map<number|string, CopterController>} */
        this._controllers = new Map();
        // Copter 1 always exists: simulator palette / project persistence read it directly.
        this.runtime._copterSimBlocks = this.getController(1);
        this.runtime._copterBlocks = this;
    }

    getPrimitives () {
        const primitives = {
            copter_with_copters: this.copter_with_copters,
            copter_value_of: this.copter_value_of
        };
        COPTER_COMMAND_OPCODES.forEach(opcode => {
            primitives[opcode] = (args, util) => this._runCommand(opcode, args, util);
        });
        Object.keys(COPTER_REPORTER_DEFAULTS).forEach(opcode => {
            primitives[opcode] = (args, util) => {
                const ids = this._targetIds(util);
                if (ids.length === 0) return COPTER_REPORTER_DEFAULTS[opcode];
                // Reporters read the first copter of the selection.
                return this.getController(ids[0])[opcode](args, util);
            };
        });
        return primitives;
    }

    getMonitored () {
        return {};
    }

    /**
     * Controller of one copter, created on demand.
     * @param {number|string} copterId 1..15 or 'new'
     * @returns {CopterController} controller
     */
    getController (copterId) {
        let controller = this._controllers.get(copterId);
        if (!controller) {
            const qca = this.runtime.QCA && typeof this.runtime.QCA.copter === 'function' ?
                this.runtime.QCA.copter(copterId) :
                null;
            controller = new CopterController(this.runtime, {copterId: copterId, qca: qca});
            this._controllers.set(copterId, controller);
        }
        return controller;
    }

    /** @returns {Array<CopterController>} controllers of the simulator copter sprites on stage */
    getSimControllers () {
        return listSimCopterNumbers(this.runtime.targets).map(n => this.getController(n));
    }

    /**
     * Copter ids that blocks can drive right now: simulator sprites, or connected enabled
     * Crazyflies.
     * @returns {Array<number|string>} ids
     */
    getActiveCopterIds () {
        if (this.runtime.sim_copter_ac) {
            return listSimCopterNumbers(this.runtime.targets);
        }
        const qca = this.runtime.QCA;
        if (!qca || typeof qca.getCopters !== 'function') return [];
        return qca.getCopters()
            .filter(c => c.connected && c.enabled)
            .map(c => c.id);
    }

    /** @returns {number|string} copter driven when nothing is active yet (primary / sim copter 1) */
    _fallbackCopterId () {
        if (this.runtime.sim_copter_ac) return 1;
        const qca = this.runtime.QCA;
        return qca && typeof qca.getCopterId === 'function' ? qca.getCopterId() : 1;
    }

    /**
     * Copters the current block acts on: the innermost `copters [N] do` selection, otherwise every
     * active copter (or the fallback copter when none is active, as for one-copter projects).
     * @param {object} util block utility
     * @returns {Array<number|string>} copter ids
     */
    _targetIds (util) {
        const stack = util && util.thread && util.thread.copterSelection;
        const selection = stack && stack.length > 0 ? stack[stack.length - 1] : ALL_COPTERS;
        if (selection === ALL_COPTERS) {
            const active = this.getActiveCopterIds();
            return active.length > 0 ? active : [this._fallbackCopterId()];
        }
        return selection;
    }

    /**
     * Run a command on every target copter; the block finishes when all of them are done.
     * Progress per copter lives in the block's stack frame across yields.
     * @param {string} opcode block opcode
     * @param {object} args block arguments
     * @param {object} util block utility
     * @returns {*} result of a single-copter command
     */
    _runCommand (opcode, args, util) {
        const ids = this._targetIds(util);
        if (ids.length === 1) {
            return this._runOnController(this.getController(ids[0]), opcode, args, util);
        }
        const frame = util.stackFrame;
        if (!frame.copterSwarm || frame.copterSwarm.opcode !== opcode) {
            frame.copterSwarm = {opcode: opcode, pending: ids.slice()};
        }
        const stillRunning = [];
        frame.copterSwarm.pending.forEach(id => {
            let yielded = false;
            const copterUtil = Object.create(util);
            copterUtil.yield = () => {
                yielded = true;
            };
            this._runOnController(this.getController(id), opcode, args, copterUtil);
            if (yielded) stillRunning.push(id);
        });
        if (stillRunning.length > 0) {
            frame.copterSwarm.pending = stillRunning;
            util.yield();
        } else {
            frame.copterSwarm = null;
        }
    }

    _runOnController (controller, opcode, args, util) {
        if (this.runtime.sim_copter_ac && controller.sim_battery <= 0 && SIM_FLIGHT_OPCODES.has(opcode)) {
            return;
        }
        return controller[opcode](args, util);
    }

    /**
     * @param {*} value block argument: a number, "1, 3", "1-3", "all" (or the factory id "new")
     * @returns {?(Array<number|string>|string)} ids, {@link ALL_COPTERS}, or null for an invalid value
     */
    _parseCopterArg (value) {
        const text = String(value).trim()
            .toLowerCase();
        if (text === ALL_COPTERS || text === 'все') return ALL_COPTERS;
        if (text === FACTORY_COPTER_ID) return [FACTORY_COPTER_ID];
        const ids = [];
        text.split(/[,;\s]+/).filter(Boolean)
            .forEach(part => {
                const range = /^(\d+)\s*-\s*(\d+)$/.exec(part);
                const from = range ? Number(range[1]) : Math.round(Cast.toNumber(part));
                const to = range ? Number(range[2]) : from;
                for (let n = Math.min(from, to); n <= Math.max(from, to); n++) {
                    if (n >= 1 && n <= MAX_SIM_COPTER_NUMBER && ids.indexOf(n) === -1) ids.push(n);
                }
            });
        return ids.length > 0 ? ids : null;
    }

    /**
     * `copters [N] do { ... }`: commands inside go to copter N (or a set / all copters). The
     * selection lives on the thread's stack (custom blocks inherit it, new threads do not);
     * re-entry after the branch pops it.
     * @param {object} args block arguments (COPTERS)
     * @param {object} util block utility
     */
    copter_with_copters (args, util) {
        const frame = util.stackFrame;
        if (frame.copterSelectionPushed) {
            frame.copterSelectionPushed = false;
            if (util.thread.copterSelection) util.thread.copterSelection.pop();
            return;
        }
        const selection = this._parseCopterArg(args.COPTERS);
        if (selection === null) return;
        if (!util.thread.copterSelection) util.thread.copterSelection = [];
        util.thread.copterSelection.push(selection);
        frame.copterSelectionPushed = true;
        util.startBranch(1, true);
    }

    /**
     * `[property ▾] of copter [N]` reads any copter regardless of the current selection.
     * @param {object} args block arguments (PROPERTY, COPTER)
     * @param {object} util block utility
     * @returns {number|boolean} property value
     */
    copter_value_of (args, util) {
        const selection = this._parseCopterArg(args.COPTER);
        const opcode = COPTER_PROPERTY_REPORTERS[String(args.PROPERTY)];
        if (selection === null || !opcode) return 0;
        const ids = selection === ALL_COPTERS ? this.getActiveCopterIds() : selection;
        if (ids.length === 0) return 0;
        return this.getController(ids[0])[opcode]({}, util);
    }
}

module.exports = Scratch3QuadcopterBlocks;

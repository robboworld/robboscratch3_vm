const test = require('tap').test;
const CopterController = require('../../src/blocks/copter-controller');
const Scratch3QuadcopterBlocks = require('../../src/blocks/scratch3_quadcopter');

class FakeRuntime {
    static get STAGE_WIDTH () {
        return 480;
    }

    static get STAGE_HEIGHT () {
        return 360;
    }

    constructor (target) {
        this.targets = [target];
        this.QCA = {
            get_coord: () => 0,
            getTelemetrySnapshot: () => ({yawDeg: 0})
        };
    }

    on () {}

    emit () {}
}

/**
 * Sprite stub: 60 px wide at 100 %, like the bundled copter costume.
 * @returns {object} fake rendered target
 */
const makeCopterTarget = function () {
    return {
        isOriginal: true,
        isStage: false,
        sprite: {name: 'Robbo Quadcopter'},
        draggable: true,
        x: 0,
        y: 0,
        size: 100,
        direction: 90,
        currentCostume: 0,
        setXY (x, y) {
            this.x = x;
            this.y = y;
        },
        setDirection (d) {
            this.direction = d;
        },
        setSize (s) {
            this.size = s;
        },
        setCostume (i) {
            this.currentCostume = i;
        },
        getBounds () {
            const half = 30 * this.size / 100;
            return {left: this.x - half, right: this.x + half, top: this.y + half, bottom: this.y - half};
        }
    };
};

/**
 * @returns {{blocks: CopterController, target: object}} copter 1 with the simulator on
 */
const makeBlocks = function () {
    const target = makeCopterTarget();
    const runtime = new FakeRuntime(target);
    const blocks = new CopterController(runtime, {copterId: 1});
    runtime.sim_copter_ac = true;
    return {blocks, target};
};

test('sim: 1 m = 100 px', t => {
    const {blocks, target} = makeBlocks();
    blocks.sim_x = 1;
    blocks.sim_y = -0.5;
    blocks._simApplyState();
    t.equal(target.x, 100);
    t.equal(target.y, -50);
    t.end();
});

test('sim: copter stays fully visible on stage', t => {
    const {blocks, target} = makeBlocks();
    blocks.sim_x = 10;
    blocks.sim_y = -10;
    blocks._simApplyState();
    t.ok(target.x + 30 <= 240, 'right edge inside stage');
    t.ok(target.y - 30 >= -180, 'bottom edge inside stage');
    t.equal(blocks.sim_x, target.x / 100, 'clamped position written back to sim state');
    t.end();
});

test('sim: user size is kept and scaled by altitude', t => {
    const {blocks, target} = makeBlocks();
    target.size = 40;
    blocks.sim_z = 0;
    blocks._simApplyState();
    t.equal(target.size, 40, 'ground size untouched');

    blocks.sim_z = 1;
    blocks._simApplyState();
    t.ok(Math.abs(target.size - 72) < 1e-9, 'grows 1.8x at 1 m');

    blocks.sim_z = 0;
    blocks._simApplyState();
    t.ok(Math.abs(target.size - 40) < 1e-9, 'returns to user size after landing');
    t.end();
});

test('sim: speed drives takeoff, landing and yaw rates', t => {
    const {blocks} = makeBlocks();
    t.equal(blocks.speed, 0.5, 'default speed matches toolbox shadow');
    const slowTakeoff = blocks._simTakeoffMps();
    const slowYaw = blocks._yawRateDps();
    blocks.speed = 1;
    t.ok(Math.abs(blocks._simTakeoffMps() - (slowTakeoff * 2)) < 1e-9, 'takeoff speed doubles');
    t.ok(Math.abs(blocks._yawRateDps() - (slowYaw * 2)) < 1e-9, 'yaw rate doubles');
    blocks.speed = 5;
    t.equal(blocks._hardwareSpeedMps(), 1, 'hardware speed is clamped');
    t.end();
});

test('sim: reaching Z=0 is a landing', t => {
    const {blocks, target} = makeBlocks();
    blocks.sim_is_flying = true;
    blocks.sim_z = 0.01;
    blocks._simEnsureFlyingFlagByAltitude();
    blocks._simApplyState();
    t.equal(blocks.sim_is_flying, false, 'not flying');
    t.equal(blocks.sim_z, 0);
    t.equal(target.currentCostume, 0, 'idle costume: propellers stopped');
    t.equal(blocks._simCanExecuteAirCommand(), false, 'air commands need takeoff');
    t.end();
});

/**
 * Thread / util stub for the C-block.
 * @returns {object} util
 */
const makeUtil = function () {
    const util = {
        thread: {},
        stackFrame: {},
        branchStarted: 0,
        startBranch () {
            util.branchStarted += 1;
        }
    };
    return util;
};

/**
 * @param {Array<string>} names sprite names on stage
 * @returns {{runtime: FakeRuntime, pkg: Scratch3QuadcopterBlocks, targets: Array<object>}} package
 */
const makeMultiCopterRuntime = function (names) {
    const targets = names.map(name => Object.assign(makeCopterTarget(), {sprite: {name}}));
    const runtime = new FakeRuntime(targets[0]);
    runtime.targets = targets;
    const pkg = new Scratch3QuadcopterBlocks(runtime);
    runtime.sim_copter_ac = true;
    return {runtime, pkg, targets};
};

test('blocks: sprites "Robbo Quadcopter" and "Robbo Quadcopter 2" are copters 1 and 2', t => {
    const {pkg} = makeMultiCopterRuntime(['Robbo Quadcopter', 'Robbo Quadcopter 2', 'Cat']);
    t.same(pkg.getActiveCopterIds(), [1, 2]);
    t.end();
});

test('blocks: copters [N] do selects the copters and restores them after the branch', t => {
    const {pkg} = makeMultiCopterRuntime(['Robbo Quadcopter', 'Robbo Quadcopter 2']);
    const util = makeUtil();
    pkg.copter_with_copters({COPTERS: 2}, util);
    t.equal(util.branchStarted, 1, 'body started');
    t.same(pkg._targetIds(util), [2], 'blocks inside go to copter 2');

    const inner = Object.assign({}, util, {stackFrame: {}});
    pkg.copter_with_copters({COPTERS: 1}, inner);
    t.same(pkg._targetIds(util), [1], 'nested block overrides');
    pkg.copter_with_copters({}, inner);
    t.same(pkg._targetIds(util), [2], 'outer selection back after nested body');

    pkg.copter_with_copters({}, util);
    t.same(util.thread.copterSelection, [], 'selection popped after the body');
    t.end();
});

test('blocks: sets, ranges and "all"', t => {
    const {pkg} = makeMultiCopterRuntime(['Robbo Quadcopter', 'Robbo Quadcopter 2', 'Robbo Quadcopter 3']);
    t.same(pkg._parseCopterArg('1, 3'), [1, 3]);
    t.same(pkg._parseCopterArg('2-3'), [2, 3]);
    t.same(pkg._parseCopterArg(2), [2], 'a variable holding a number');
    t.equal(pkg._parseCopterArg('all'), 'all');
    t.equal(pkg._parseCopterArg('99'), null);
    const util = makeUtil();
    pkg.copter_with_copters({COPTERS: 'all'}, util);
    t.same(pkg._targetIds(util), [1, 2, 3]);
    t.end();
});

test('blocks: without a selection every active copter is driven', t => {
    const {pkg} = makeMultiCopterRuntime(['Robbo Quadcopter', 'Robbo Quadcopter 2']);
    t.same(pkg._targetIds(makeUtil()), [1, 2]);
    t.end();
});

test('blocks: one copter and no selection — old projects keep working', t => {
    const {pkg} = makeMultiCopterRuntime(['Robbo Quadcopter']);
    t.same(pkg._targetIds(makeUtil()), [1]);
    t.end();
});

test('blocks: a command for several copters waits for all of them', t => {
    const {pkg} = makeMultiCopterRuntime(['Robbo Quadcopter', 'Robbo Quadcopter 2']);
    const calls = [];
    const remaining = {1: 1, 2: 3};
    [1, 2].forEach(n => {
        pkg.getController(n).copter_set_speed = (args, u) => {
            calls.push(n);
            if (remaining[n]-- > 0) u.yield();
        };
    });
    const util = Object.assign(makeUtil(), {yields: 0});
    util.yield = () => {
        util.yields += 1;
    };
    for (let step = 0; step < 4; step++) pkg._runCommand('copter_set_speed', {}, util);
    t.same(calls, [1, 2, 1, 2, 2, 2], 'finished copter 1 is not restarted');
    t.equal(util.yields, 3, 'block yields until copter 2 is done');
    t.end();
});

test('blocks: [X] of copter [N] reads any copter', t => {
    const {pkg} = makeMultiCopterRuntime(['Robbo Quadcopter', 'Robbo Quadcopter 2']);
    const copter2 = pkg.getController(2);
    copter2.sim_x = 1.25;
    copter2._simApplyState();
    t.equal(pkg.copter_value_of({PROPERTY: 'x', COPTER: 2}, makeUtil()), 1.25);
    t.equal(pkg.copter_value_of({PROPERTY: 'x', COPTER: 99}, makeUtil()), 0, 'invalid copter');
    t.end();
});

test('sim battery: drains in flight, lands at 0 %, charge block refills', t => {
    const {blocks} = makeBlocks();
    blocks.sim_is_flying = true;
    blocks.sim_z = 0.5;
    blocks.sim_battery = 1;
    blocks._simBatteryLastTick = Date.now() - 6000;
    blocks._simBatteryTick();
    t.equal(blocks.sim_battery, 0, '6 s at 1 % / 3.6 s empties the battery');
    t.ok(blocks.sim_interval, 'descent started');
    blocks._simClearInterval();
    blocks.copter_charge_battery({PERCENT: 150});
    t.equal(blocks.sim_battery, 100, 'clamped to 100');
    blocks.sim_is_flying = false;
    blocks._simSyncBatteryTimer();
    t.notOk(blocks._simBatteryTimer, 'no drain on the ground');
    t.end();
});

test('sim: each copter has its own start point, like a Flow deck', t => {
    const {blocks, target} = makeBlocks();
    target.x = -160;
    blocks.syncFromSpritePosition();
    t.equal(blocks.sim_origin_x, -1.6, 'placed on the ground: start point follows the sprite');
    t.equal(blocks.sim_x, 0, 'local coordinates stay at the start');
    blocks.sim_x = 0.5;
    blocks._simApplyState();
    t.ok(Math.abs(target.x + 110) < 1e-6, 'local 0.5 m from its own start');
    blocks.sim_x = 0.3;
    blocks.copter_reset_local_position();
    t.equal(blocks.sim_x, 0, 'reset: current place is the new start');
    t.ok(Math.abs(target.x + 130) < 1e-6, 'sprite does not jump');
    t.end();
});

test('takeoff climbs to an absolute 0.3 m and does nothing in the air', t => {
    const {blocks} = makeBlocks();
    blocks._ensureSimCopterSprite = () => true;
    blocks.sim_is_flying = true;
    blocks.sim_z = 0.6;
    const util = Object.assign(makeUtil(), {yield () {}});
    blocks.copter_fly_up({}, util);
    t.equal(blocks.sim_z, 0.6, 'sim: a second takeoff keeps the height');
    t.equal(blocks.fack, 0, 'sim: finished at once');

    // Hardware: in the air the block returns without scheduling an HL takeoff.
    let takeoffs = 0;
    const qca = {
        isQuadcopterConnected: () => true,
        isQuadcopterFlying: () => true,
        takeoff: () => {
            takeoffs += 1;
        },
        get_coord: () => 0.6
    };
    const hwRuntime = new FakeRuntime(makeCopterTarget());
    const hw = new CopterController(hwRuntime, {copterId: 1, qca});
    hw.copter_fly_up({}, util);
    t.equal(takeoffs, 0, 'hardware: no takeoff in the air');
    blocks.sim_is_flying = false;
    blocks._simSyncBatteryTimer();
    t.end();
});

test('flight blocks are refused on the ground without charge for a flight', t => {
    let takeoffs = 0;
    const qca = {
        isQuadcopterConnected: () => true,
        isQuadcopterFlying: () => false,
        getBatteryState: () => ({level: 'ok', percent: 10, minutes: 0, canTakeOff: false}),
        takeoff: () => {
            takeoffs += 1;
        },
        get_coord: () => 0
    };
    const hw = new CopterController(new FakeRuntime(makeCopterTarget()), {copterId: 1, qca});
    t.equal(hw._refuseForCriticalBattery('copter_fly_up'), true, 'takeoff refused');
    t.equal(hw._refuseForCriticalBattery('copter_land'), false, 'landing always allowed');
    hw.copter_fly_up({}, Object.assign(makeUtil(), {yield () {}}));
    t.equal(takeoffs, 0);
    t.end();
});

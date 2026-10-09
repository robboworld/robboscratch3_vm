const test = require('tap').test;
const Thread = require('../../src/engine/thread');
const CopterController = require('../../src/blocks/copter-controller');

class FakeRuntime {
    constructor () {
        this.targets = [];
        this.threads = [];
        this.sim_copter_ac = false;
    }

    on () {}

    emit () {}
}

/**
 * QCA stub of a real copter: pose from the test, every flight command recorded.
 * @param {object} pose x, y, z (m) and yawDeg of the copter; missing values default to hovering at 0.5 m
 * @returns {object} fake QCA
 */
const makeQca = function (pose) {
    const calls = {goTo: [], land: 0, hold: 0, takeoff: 0};
    return {
        pose: Object.assign({x: 0, y: 0, z: 0.5, yawDeg: 0}, pose),
        calls,
        isQuadcopterConnected: () => true,
        isQuadcopterFlying () {
            return this.pose.z > 0.05;
        },
        isSafetyLanding: () => false,
        getBatteryState: () => ({level: 'ok', canTakeOff: true}),
        get_coord (axis) {
            return {X: this.pose.x, Y: this.pose.y, Z: this.pose.z}[axis] || 0;
        },
        getTelemetrySnapshot () {
            return {x: this.pose.x, y: this.pose.y, z: this.pose.z, yawDeg: this.pose.yawDeg};
        },
        get_x_telemetry_delta: () => 0,
        get_y_telemetry_delta: () => 0,
        goToHighLevel (dx, dy, dz, yawRad, durationS, options) {
            calls.goTo.push({dx, dy, dz, yawRad, durationS, options});
        },
        takeoff: () => {
            calls.takeoff += 1;
        },
        landAndClose: () => {
            calls.land += 1;
        },
        computeHlLandWaitMs: () => 1000,
        holdPosition: () => {
            calls.hold += 1;
        },
        hoverStop () {
            this.holdPosition();
        }
    };
};

const makeCopter = function (t, pose) {
    const runtime = new FakeRuntime();
    const qca = makeQca(pose);
    const copter = new CopterController(runtime, {copterId: 1, qca});
    // A block still running keeps its dispatch timer: stop it so the test process can exit.
    t.teardown(() => copter.commandCoordinator.cancel('projectStopAll'));
    return {runtime, qca, copter};
};

/**
 * Util of a running script (its thread is alive in the runtime).
 * @param {FakeRuntime} runtime runtime
 * @param {string} topBlock hat id
 * @returns {object} block util
 */
const makeUtil = function (runtime, topBlock) {
    const thread = new Thread(topBlock);
    runtime.threads.push(thread);
    return {thread, yield: () => {}};
};

const near = (t, actual, expected, message) => t.ok(Math.abs(actual - expected) < 1e-6,
    `${message}: ${actual} ≈ ${expected}`);

test('fly forward N seconds after a 90° turn: one go_to along the nose, in the world frame', t => {
    const {runtime, qca, copter} = makeCopter(t, {yawDeg: 90});
    const speed = copter._hardwareSpeedMps();
    copter.copter_fly_time({DIRECTION: 'forward', SECONDS: 2}, makeUtil(runtime, 'a'));
    t.equal(qca.calls.goTo.length, 1, 'one HL command');
    const move = qca.calls.goTo[0];
    near(t, move.dx, 0, 'world dx');
    near(t, move.dy, speed * 2, 'world dy');
    near(t, move.dz, 0, 'height kept');
    near(t, move.durationS, 2, 'over the block time');
    t.ok(move.options.relative && move.options.forceLegacy, 'relative legacy go_to');
    t.end();
});

test('fly N meters after a turn goes where the nose points', t => {
    const {runtime, qca, copter} = makeCopter(t, {yawDeg: 90});
    copter.copter_fly_distance({DIRECTION: 'forward', METERS: 1}, makeUtil(runtime, 'a'));
    near(t, qca.calls.goTo[0].dx, 0, 'world dx');
    near(t, qca.calls.goTo[0].dy, 1, 'world dy');
    t.end();
});

test('turn left 90°: yaw offset +π/2 only', t => {
    const {runtime, qca, copter} = makeCopter(t);
    copter.copter_rotate({SIDE: 'left', DEGREES: 90}, makeUtil(runtime, 'a'));
    const turn = qca.calls.goTo[0];
    near(t, turn.yawRad, Math.PI / 2, 'yaw');
    near(t, Math.hypot(turn.dx, turn.dy, turn.dz), 0, 'no displacement');
    t.end();
});

test('change x by 0.2: 0.2 m along world X, no 3 s minimum', t => {
    const {runtime, qca, copter} = makeCopter(t, {yawDeg: 45});
    copter.copter_change_x_by({DISTANCE_DELTA: 0.2}, makeUtil(runtime, 'a'));
    const move = qca.calls.goTo[0];
    near(t, move.dx, 0.2, 'dx');
    near(t, move.dy, 0, 'dy');
    t.ok(move.durationS < 1, `short flight: ${move.durationS} s`);
    t.end();
});

test('change z by −0.4 from 0.5 m: down to 0.1 m, no landing', t => {
    const {runtime, qca, copter} = makeCopter(t, {z: 0.5});
    const util = makeUtil(runtime, 'a');
    copter.copter_change_z_by({DISTANCE_DELTA: -0.4}, util);
    near(t, qca.calls.goTo[0].dz, -0.4, 'dz');
    qca.pose.z = 0.45;
    copter.copter_change_z_by({DISTANCE_DELTA: -0.4}, util);
    t.equal(qca.calls.land, 0, 'not a landing on the way down');
    t.end();
});

test('move blocks do nothing on the ground', t => {
    const {runtime, qca, copter} = makeCopter(t, {z: 0});
    copter.copter_fly_time({DIRECTION: 'forward', SECONDS: 1}, makeUtil(runtime, 'a'));
    copter.copter_change_z_by({DISTANCE_DELTA: 0.5}, makeUtil(runtime, 'b'));
    t.equal(qca.calls.goTo.length, 0);
    t.notOk(copter.commandCoordinator.activeCommand);
    t.end();
});

test('a moving copter holds nothing after the block: the planner keeps the point', t => {
    const {runtime, qca, copter} = makeCopter(t);
    const util = makeUtil(runtime, 'a');
    copter.copter_change_x_by({DISTANCE_DELTA: 0.3}, util);
    qca.pose.x = 0.3;
    copter.copter_change_x_by({DISTANCE_DELTA: 0.3}, util);
    t.notOk(copter.commandCoordinator.activeCommand, 'finished at the point');
    t.equal(qca.calls.hold, 0, 'no hold / stream after a normal finish');
    t.end();
});

test('land from another script interrupts a running move', t => {
    const {runtime, qca, copter} = makeCopter(t);
    copter.copter_fly_time({DIRECTION: 'forward', SECONDS: 5}, makeUtil(runtime, 'loop'));
    copter.copter_land({}, makeUtil(runtime, 'spaceKey'));
    t.equal(qca.calls.land, 1, 'land sent at once');
    t.equal(copter.commandCoordinator.activeCommand.key, 'copter_land');
    t.equal(qca.calls.hold, 0, 'no hold before the land');
    t.end();
});

test('a move of a script that is gone is cancelled by the next one', t => {
    const {runtime, qca, copter} = makeCopter(t);
    const old = makeUtil(runtime, 'a');
    copter.copter_fly_time({DIRECTION: 'forward', SECONDS: 5}, old);
    runtime.threads.splice(runtime.threads.indexOf(old.thread), 1);
    copter.copter_fly_time({DIRECTION: 'backward', SECONDS: 1}, makeUtil(runtime, 'b'));
    t.equal(qca.calls.hold, 1, 'the orphan move is stopped');
    t.equal(qca.calls.goTo.length, 2, 'the new move starts');
    t.equal(copter.commandCoordinator.activeCommand.ownerId, 'b');
    t.end();
});

test('critical battery during a move drops the block without flying on', t => {
    const {runtime, qca, copter} = makeCopter(t);
    const util = makeUtil(runtime, 'a');
    copter.copter_fly_time({DIRECTION: 'forward', SECONDS: 5}, util);
    qca.getBatteryState = () => ({level: 'critical', canTakeOff: false});
    copter.copter_fly_time({DIRECTION: 'forward', SECONDS: 5}, util);
    t.notOk(copter.commandCoordinator.activeCommand, 'block dropped');
    t.equal(qca.calls.hold, 0, 'the DCA lands it: no hold');
    t.equal(qca.calls.goTo.length, 1);
    t.end();
});

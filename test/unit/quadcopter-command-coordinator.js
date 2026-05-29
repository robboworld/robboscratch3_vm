const test = require('tap').test;
const Thread = require('../../src/engine/thread');
const QuadcopterCommandCoordinator = require('../../src/blocks/quadcopter-command-coordinator');

function makeUtil (topBlock) {
    const thread = new Thread(topBlock);
    return {
        thread: thread,
        yield: () => {}
    };
}

test('coordinator: only owner thread continues active command', t => {
    const cleanupReasons = [];
    const coordinator = new QuadcopterCommandCoordinator({
        onFlightCleanup: reason => cleanupReasons.push(reason)
    });

    const utilA = makeUtil('hatA');
    const utilB = makeUtil('hatB');
    let finishCount = 0;

    coordinator.runTargetCommand('copter_fly_time', utilA, {
        timeoutMs: 5000,
        start: () => ({tick: 0}),
        shouldFinish: () => false,
        finish: () => { finishCount += 1; },
        cancel: () => {}
    });

    t.ok(coordinator.activeCommand, 'command started for hatA');
    t.equal(coordinator.activeCommand.ownerId, 'hatA');

    coordinator.runTargetCommand('copter_fly_time', utilB, {
        timeoutMs: 5000,
        start: () => ({}),
        shouldFinish: () => true,
        finish: () => { finishCount += 1; }
    });

    t.equal(coordinator.activeCommand.ownerId, 'hatA', 'foreign thread does not replace owner');
    t.equal(finishCount, 0);
    t.equal(cleanupReasons.length, 0);

    t.end();
});

test('coordinator: replaced command triggers soft cleanup hook', t => {
    const cleanupCalls = [];
    const coordinator = new QuadcopterCommandCoordinator({
        onFlightCleanup: (reason, command) => cleanupCalls.push({ reason, key: command && command.key })
    });

    const util = makeUtil('hatA');

    coordinator.runTargetCommand('cmd_a', util, {
        timeoutMs: 5000,
        start: () => ({}),
        shouldFinish: () => false,
        cancel: () => {}
    });

    coordinator.runTargetCommand('cmd_b', util, {
        timeoutMs: 5000,
        start: () => ({}),
        shouldFinish: () => false,
        cancel: () => {}
    });

    t.equal(cleanupCalls.length, 1);
    t.equal(cleanupCalls[0].reason, 'replaced');
    t.equal(cleanupCalls[0].key, 'cmd_a');
    t.equal(coordinator.activeCommand.key, 'cmd_b');
    t.end();
});

test('coordinator: projectStopAll cleanup hook is skipped', t => {
    const cleanupReasons = [];
    const coordinator = new QuadcopterCommandCoordinator({
        onFlightCleanup: reason => cleanupReasons.push(reason)
    });

    const util = makeUtil('hatA');
    coordinator.runTargetCommand('cmd_a', util, {
        timeoutMs: 5000,
        start: () => ({}),
        shouldFinish: () => false,
        cancel: () => {}
    });

    coordinator.cancel('projectStopAll');
    t.equal(cleanupReasons.length, 0);
    t.notOk(coordinator.activeCommand);
    t.end();
});

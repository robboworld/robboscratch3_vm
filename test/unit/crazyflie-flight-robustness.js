const fs = require('fs');
const path = require('path');
const test = require('tap').test;
const babel = require('@babel/core');

function loadEsModuleAsCommonjs (relativePathFromRepo) {
    const filePath = path.resolve(__dirname, relativePathFromRepo);
    const source = fs.readFileSync(filePath, 'utf8');
    const {code} = babel.transformSync(source, {
        filename: filePath,
        presets: [['@babel/preset-env', {modules: 'commonjs'}]]
    });
    const module = {exports: {}};
    // eslint-disable-next-line no-new-func
    const fn = new Function('module', 'exports', 'require', code);
    fn(module, module.exports, require);
    return module.exports;
}

const gateModule = loadEsModuleAsCommonjs(
    '../../../RobboScratch3_DeviceControlAPI/src/crazyflie/flight-command-gate.js'
);
const FlightCommandGate = gateModule.default;
const {FLIGHT_COMMAND_MODE} = gateModule;

test('flight gate: rejects streaming while HL active', t => {
    const gate = new FlightCommandGate();
    gate.beginHl();
    t.notOk(gate.beginStreaming());
    t.equal(gate.stats.rejectedStreaming, 1);
    t.end();
});

test('flight gate: generation invalidates stale async token', t => {
    const gate = new FlightCommandGate();
    const token = gate.currentGeneration();
    gate.enterEmergency();
    t.ok(gate.isStale(token));
    t.notOk(gate.isStale(gate.currentGeneration()));
    t.end();
});

test('flight gate: LOST bumps generation and blocks HL', t => {
    const gate = new FlightCommandGate();
    const token = gate.currentGeneration();
    gate.enterLost();
    t.ok(gate.isStale(token));
    t.notOk(gate.canBeginHl());
    t.equal(gate.mode, FLIGHT_COMMAND_MODE.LOST);
    t.end();
});

test('flight gate: emergency reset to idle with force', t => {
    const gate = new FlightCommandGate();
    gate.enterEmergency();
    gate.trySetMode(FLIGHT_COMMAND_MODE.IDLE, {force: true});
    t.equal(gate.mode, FLIGHT_COMMAND_MODE.IDLE);
    t.end();
});

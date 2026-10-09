const fs = require('fs');
const path = require('path');
const test = require('tap').test;
const babel = require('@babel/core');

const DCA_SRC = path.resolve(__dirname, '../../../RobboScratch3_DeviceControlAPI/src');
const moduleCache = {};

/**
 * Transpile a DCA ES module (and its relative imports) to CommonJS.
 * @param {string} filePath absolute path
 * @returns {object} module exports
 */
const loadDcaModule = function (filePath) {
    const resolved = filePath.endsWith('.js') ? filePath : `${filePath}.js`;
    if (moduleCache[resolved]) return moduleCache[resolved].exports;
    const source = fs.readFileSync(resolved, 'utf8');
    const {code} = babel.transformSync(source, {
        filename: resolved,
        presets: [['@babel/preset-env', {modules: 'commonjs', targets: {node: 'current'}}]]
    });
    const module = {exports: {}};
    moduleCache[resolved] = module;
    const localRequire = spec => (spec.startsWith('.') ?
        loadDcaModule(path.resolve(path.dirname(resolved), spec)) :
        require(spec)); // eslint-disable-line global-require
    // eslint-disable-next-line no-new-func
    const fn = new Function('module', 'exports', 'require', code);
    fn(module, module.exports, localRequire);
    return module.exports;
};

const protocol = loadDcaModule(path.join(DCA_SRC, 'crazyflie/protocol.js'));
const telemetryProfile = loadDcaModule(path.join(DCA_SRC, 'crazyflie/telemetry-profile.js'));
const CrazyflieSession = loadDcaModule(path.join(DCA_SRC, 'crazyflie/session.js')).default;

/**
 * Link stub recording every packet as a byte array.
 * @returns {object} fake CrazyradioLink
 */
const makeFakeLink = function () {
    const sent = [];
    const reply = packet => {
        sent.push(Array.from(new Uint8Array(packet)));
        return Promise.resolve({state: true, data: new Uint8Array(0)});
    };
    return {
        sent,
        isOpen: () => true,
        open: () => Promise.resolve(true),
        close: () => Promise.resolve({reason: 'closed'}),
        flushQueue: () => {},
        getStats: () => ({}),
        sendPacket: reply,
        sendPacketUrgent: reply
    };
};

const makeConnectedSession = function () {
    const link = makeFakeLink();
    const session = new CrazyflieSession({link});
    session.status = Object.assign({}, session.status, {connected: true, state: 'connected'});
    return {session, link};
};

test('protocol: supervisor bitfield response is decoded', t => {
    // Port 9 / channel 0, response to 0x0C, bitfield: canFly (3) + isLocked (6) + hlTrajFinished (9).
    const bitfield = (1 << 3) | (1 << 6) | (1 << 9);
    const data = new Uint8Array([0x9c, 0x8c, bitfield & 0xff, bitfield >> 8]);
    const sup = protocol.parseSupervisorStateResponse(data);
    t.ok(sup.canFly);
    t.ok(sup.isLocked);
    t.ok(sup.hlTrajFinished);
    t.notOk(sup.isFlying);
    const logData = new Uint8Array([0x52, 0x00, 0x00]);
    t.equal(protocol.parseSupervisorStateResponse(logData), null, 'log data is not supervisor');
    t.end();
});

test('protocol: nRF power packets match crazyflie2-nrf-firmware', t => {
    const bytes = Array.from(new Uint8Array(protocol.buildNrfPowerPacket(protocol.NRF_POWER_CMD.STM_OFF)));
    t.same(bytes, [0xf3, 0xfe, 0x02]);
    t.end();
});

test('protocol: only port 5 channel 2 is telemetry', t => {
    t.ok(protocol.isTelemetryPayload(new Uint8Array([0x52, 0, 0, 0, 0])));
    t.ok(protocol.isTelemetryPayload(new Uint8Array([0x5e, 0, 0, 0, 0])), 'link bits ignored');
    t.notOk(protocol.isTelemetryPayload(new Uint8Array([0x50, 0, 0, 0, 0])), 'TOC channel');
    t.notOk(protocol.isTelemetryPayload(new Uint8Array([0x54, 0, 0, 0, 0])), 'TOC channel with link bit');
    t.end();
});

test('telemetry: non-float fields keep following fields aligned', t => {
    const profile = {
        blocks: [{blockId: 1, fields: [{name: 'state', type: 4}, {name: 'vbat', type: 7}]}]
    };
    // header, block id, 3-byte timestamp, int8 = -1, float32 = 3.7
    const buf = new ArrayBuffer(10);
    const view = new DataView(buf);
    view.setUint8(0, 0x52);
    view.setUint8(1, 1);
    view.setInt8(5, -1);
    view.setFloat32(6, 3.7, true);
    const decoded = telemetryProfile.decodeTelemetryPacket(profile, new Uint8Array(buf));
    t.equal(decoded.values.state, -1);
    t.ok(Math.abs(decoded.values.vbat - 3.7) < 1e-6);
    t.end();
});

test('session: hoverStop on the ground does not stream setpoints', t => {
    const {session, link} = makeConnectedSession();
    session.telemetryStore.getValue = () => 0;
    return session.hoverStop().then(() => {
        t.notOk(session.velocityCommandLoop, 'no refresh loop');
        t.notOk(link.sent.some(p => p[0] === 0x7c), 'no velocity setpoint sent');
        t.end();
    });
});

test('session: search is refused while airborne', t => {
    const {session} = makeConnectedSession();
    session.telemetryStore.getValue = () => 0.6;
    session.telemetryStore.hasReceivedField = () => true;
    return session.search().then(ok => {
        t.equal(ok, false);
        t.equal(session.status.lastError.code, 'searchRefusedInFlight');
        t.end();
    });
});

test('session: takeoff is refused while the supervisor is locked', t => {
    const {session} = makeConnectedSession();
    session.supervisor = {isLocked: true};
    return session.takeoffHighLevel(0.3, 1).then(ok => {
        t.equal(ok, false);
        t.equal(session.status.lastError.code, 'copterLocked');
        t.end();
    });
});

test('session: upright after a crash sends recovery automatically', t => {
    const {session, link} = makeConnectedSession();
    session._applySupervisorState({bitfield: 1 << 7, isCrashed: true, isTumbled: false, isLocked: false});
    return Promise.resolve().then(() => {
        t.ok(link.sent.some(p => p[0] === 0x9d && p[1] === protocol.SUPERVISOR_CMD_RECOVER), 'recover sent');
        t.ok(session._needsRearmBeforeTakeoff, 'arming deferred to takeoff');
        t.end();
    });
});

test('session: reboot power-cycles the STM32 then searches again', t => {
    const {session, link} = makeConnectedSession();
    session.telemetryStore.getValue = () => 0;
    let searched = false;
    session.search = () => {
        searched = true;
        return Promise.resolve(true);
    };
    return session.rebootCopter().then(ok => {
        const power = link.sent.filter(p => p[0] === 0xf3 && p[1] === 0xfe).map(p => p[2]);
        t.same(power, [0x02, 0x03], 'STM off, then on');
        t.ok(searched);
        t.equal(ok, true);
        t.end();
    });
});

test('session: power off sends nRF ALL_OFF and closes the link without searching', t => {
    const {session, link} = makeConnectedSession();
    session.telemetryStore.getValue = () => 0;
    let searched = false;
    session.search = () => {
        searched = true;
        return Promise.resolve(true);
    };
    return session.powerOffCopter().then(ok => {
        const power = link.sent.filter(p => p[0] === 0xf3 && p[1] === 0xfe).map(p => p[2]);
        t.same(power, [0x01], 'all off');
        t.equal(ok, true);
        t.notOk(searched);
        t.equal(session.getStatusSnapshot().state, 'disconnected');
        t.equal(session.getStatusSnapshot().closeReason, 'poweredOff');
        t.end();
    });
});

test('session: power off is refused in the air', t => {
    const {session, link} = makeConnectedSession();
    session.isAirborne = () => true;
    return session.powerOffCopter().then(ok => {
        t.equal(ok, false);
        t.equal(link.sent.length, 0);
        t.end();
    });
});

test('session: a failed config write is tried once more', t => {
    const {session} = makeConnectedSession();
    session.telemetryStore.getValue = () => 0;
    let calls = 0;
    session._writeRadioConfigOnce = () => {
        calls += 1;
        if (calls === 1) return Promise.reject(new Error('Crazyflie control response did not arrive'));
        return Promise.resolve(true);
    };
    return session.writeRadioConfig({channel: 92, address: new Uint8Array(5)}).then(ok => {
        t.equal(ok, true);
        t.equal(calls, 2);
        t.end();
    });
});

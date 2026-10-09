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
const CrazyflieSession = loadDcaModule(path.join(DCA_SRC, 'crazyflie/session.js')).default;

const HL_HEADER = 0x8c;
const NOTIFY_HEADER = 0x7d;
const VELOCITY_HEADER = 0x7c;

const isHl = (packet, command) => packet[0] === HL_HEADER && packet[1] === command;
const isNotify = packet => packet[0] === NOTIFY_HEADER && packet[1] === 0;
const isHoverSetpoint = packet => packet[0] === VELOCITY_HEADER && packet[1] === 5;
const wait = ms => new Promise(resolve => setTimeout(resolve, ms));

/**
 * Link stub recording every packet; `ack(packet)` decides whether the copter answered.
 * @param {function(Array<number>): boolean} [ack] default: every packet is acknowledged
 * @returns {object} fake CrazyradioLink
 */
const makeFakeLink = function (ack) {
    const sent = [];
    const reply = packet => {
        const bytes = Array.from(new Uint8Array(packet));
        sent.push(bytes);
        const ok = ack ? ack(bytes) : true;
        return Promise.resolve({state: ok, data: new Uint8Array(0)});
    };
    return {
        sent,
        quality: 100,
        isOpen: () => true,
        open: () => Promise.resolve(true),
        close: () => Promise.resolve({reason: 'closed'}),
        flushQueue: () => {},
        getStats: () => ({}),
        getLinkQuality () {
            return this.quality;
        },
        sendPacket: reply,
        sendPacketUrgent: reply
    };
};

const setHeight = function (session, z) {
    session.telemetryStore.values.z = z;
    session.telemetryStore.receivedFields.z = true;
};

/**
 * Connected session hovering at 0.5 m.
 * @param {function(Array<number>): boolean} [ack] see {@link makeFakeLink}
 * @returns {{session: CrazyflieSession, link: object}} the session and its link stub
 */
const makeFlyingSession = function (ack) {
    const link = makeFakeLink(ack);
    const session = new CrazyflieSession({link});
    session.status = Object.assign({}, session.status, {connected: true, state: 'connected'});
    setHeight(session, 0.5);
    return {session, link};
};

/**
 * Cancel the HL work still waiting on telemetry so the test process can exit.
 * @param {CrazyflieSession} session session under test
 * @returns {Promise} resolves once the waits saw the cancellation
 */
const settle = session => {
    session._invalidateFlightCommands();
    return wait(120);
};

test('hold in the air: go_to (0, 0, 0) first, notify after, no setpoint stream', t => {
    const {session, link} = makeFlyingSession();
    return session.moveWithSpeed(0.2, 0, 0, 0.5)
        .then(() => session.hoverStop())
        .then(() => wait(400))
        .then(() => {
            const goTo = link.sent.findIndex(p => isHl(p, protocol.HL_COMMAND_GO_TO));
            const notify = link.sent.findIndex((p, i) => i > goTo && isNotify(p));
            t.ok(goTo >= 0, 'relative go_to sent');
            t.equal(link.sent[goTo][3], 1, 'go_to is relative');
            t.ok(notify > goTo, 'notify_setpoint_stop only after the go_to');
            t.notOk(link.sent.slice(goTo).some(isHoverSetpoint), 'no hover setpoint after the handover');
            t.notOk(link.sent.some(p => isHl(p, protocol.HL_COMMAND_STOP)), 'no HL STOP');
            t.notOk(session.velocityCommandLoop, 'stream stopped');
            return settle(session);
        });
});

test('aborting an HL command in the air holds instead of HL STOP', t => {
    const {session, link} = makeFlyingSession();
    session.goToHighLevel(0.5, 0, 0, 0, 2, {relative: true, forceLegacy: true});
    return wait(30)
        .then(() => session.abortHlTrajectories('replaced'))
        .then(() => {
            t.notOk(link.sent.some(p => isHl(p, protocol.HL_COMMAND_STOP)), 'no HL STOP in the air');
            const goTos = link.sent.filter(p => isHl(p, protocol.HL_COMMAND_GO_TO));
            t.equal(goTos.length, 2, 'the move, then the hold');
            return settle(session);
        });
});

test('land over a running go_to replaces it without HL STOP', t => {
    const {session, link} = makeFlyingSession();
    session.goToHighLevel(0.5, 0, 0, 0, 2, {relative: true, forceLegacy: true});
    return wait(30)
        .then(() => {
            session.landAndClose();
            return wait(60);
        })
        .then(() => {
            t.ok(link.sent.some(p => isHl(p, protocol.HL_COMMAND_LAND_2)), 'land sent');
            t.notOk(link.sent.some(p => isHl(p, protocol.HL_COMMAND_STOP)), 'no HL STOP');
            return settle(session);
        });
});

test('HL packet without ACK: retried, priority not relaxed, stream kept', t => {
    const {session, link} = makeFlyingSession(p => !isHl(p, protocol.HL_COMMAND_LAND_2));
    return session.moveWithSpeed(0, 0, 0, 0.5)
        .then(() => session.landAndClose())
        .then(ok => {
            t.equal(ok, false, 'land not delivered');
            const lands = link.sent.filter(p => isHl(p, protocol.HL_COMMAND_LAND_2));
            t.ok(lands.length >= 5, `land retried (${lands.length} times within ~1 s)`);
            const firstLand = link.sent.findIndex(p => isHl(p, protocol.HL_COMMAND_LAND_2));
            t.notOk(link.sent.slice(firstLand).some(isNotify), 'no notify without the land');
            t.ok(session.velocityCommandLoop, 'hover stream resumed');
            session._clearVelocityLoop();
            return settle(session);
        });
});

test('emergency stop during takeoff preparation: no takeoff packet', t => {
    const link = makeFakeLink();
    const session = new CrazyflieSession({link});
    session.status = Object.assign({}, session.status, {connected: true, state: 'connected'});
    setHeight(session, 0);
    const takeoff = session.takeoffHighLevel(0.3, 1);
    return wait(100)
        .then(() => session.emergencyStop({keepConnected: true}))
        .then(() => takeoff)
        .then(ok => {
            t.equal(ok, false);
            t.notOk(link.sent.some(p => isHl(p, protocol.HL_COMMAND_TAKEOFF_2)), 'takeoff not sent');
            t.end();
        });
});

test('weak link in the air for 2 s: landing with weakLinkLanding', t => {
    const {session, link} = makeFlyingSession();
    link.quality = 10;
    session._watchFlightSafety();
    t.notOk(link.sent.some(p => isHl(p, protocol.HL_COMMAND_LAND_2)), 'not at once');
    session._weakLinkSince = Date.now() - 2500;
    session._watchFlightSafety();
    return wait(60).then(() => {
        t.equal(session.status.lastError.code, 'weakLinkLanding');
        t.ok(link.sent.some(p => isHl(p, protocol.HL_COMMAND_LAND_2)), 'land sent');
        t.ok(session.isSafetyLanding(), 'scripts are kept off meanwhile');
        return session.takeoffHighLevel(0.3, 1);
    })
        .then(ok => {
            t.equal(ok, false, 'takeoff refused while landing');
            return settle(session);
        });
});

test('link lost in the air: still flying, searched, landed once it answers', t => {
    const {session, link} = makeFlyingSession();
    const lands = () => link.sent.filter(p => isHl(p, protocol.HL_COMMAND_LAND_2)).length;
    session._handleLinkLost('telemetryTimeout');
    t.notOk(session.isConnected());
    t.ok(session.isLostInAir(), 'remembered as lost in the air');
    t.ok(session.isAirborne(), 'counts as flying while lost');

    // The reconnect: link up again, telemetry empty until the copter reports its height.
    session.status = Object.assign({}, session.status, {
        connected: true,
        state: 'connected',
        connectionState: 'telemetryActive'
    });
    session._flightGate.trySetMode('idle', {force: true});
    session.telemetryStore.reset();
    t.ok(session.isAirborne(), 'flying until the height is known');
    // The loss itself already tried a land: count only what comes after the reconnect.
    return wait(60).then(() => {
        const landsAtLoss = lands();
        session._watchFlightSafety();
        t.equal(lands(), landsAtLoss, 'waits for telemetry');
        setHeight(session, 0.6);
        session._watchFlightSafety();
        return wait(60).then(() => landsAtLoss);
    })
        .then(landsAtLoss => {
            t.equal(session.status.lastError.code, 'landedAfterLinkLoss');
            t.ok(lands() > landsAtLoss, 'land sent after the reconnect');
            t.notOk(session.isLostInAir());
            return settle(session);
        });
});

test('closed link while connected is a loss', t => {
    const {session, link} = makeFlyingSession();
    link.isOpen = () => false;
    session._startTelemetryPolling();
    return wait((session.pollIntervalMs * 2) + 20).then(() => {
        session._clearPollingLoop();
        t.equal(session.status.state, 'lost');
        t.equal(session.status.lostReason, 'linkClosed');
        t.end();
    });
});

test('journal: commands, deliveries and failures are recorded', t => {
    const {session} = makeFlyingSession(p => !isHl(p, protocol.HL_COMMAND_LAND_2));
    return session.goToHighLevel(0.25, 0, 0, 0, 0.5, {relative: true, forceLegacy: true})
        .then(() => session.landAndClose())
        .then(() => {
            const events = session.getFlightJournal().map(e => (e.command ? `${e.event}:${e.command}` : e.event));
            t.ok(events.indexOf('goTo') !== -1, 'go_to asked');
            t.ok(events.indexOf('delivered:goTo') !== -1, 'go_to delivered');
            t.ok(events.indexOf('notDelivered:land') !== -1, 'land not delivered');
            const failed = session.getFlightJournal().find(e => e.event === 'notDelivered');
            t.ok(failed.attempts >= 5 && failed.ms >= 1000, `${failed.attempts} attempts over ${failed.ms} ms`);
            return settle(session);
        });
});

test('radio answers but the log stopped: lost at once (telemetryStale)', t => {
    const {session, link} = makeFlyingSession();
    const now = Date.now();
    session.telemetryStore.lastPacketAt = now - 1600;
    session._lastAckOkAt = now - 100;
    t.equal(session._telemetryLossReason(now), 'telemetryStale');
    session._lastAckOkAt = now - 2000;
    t.equal(session._telemetryLossReason(now), null, 'no answers either: wait for the usual timeout');
    session.telemetryStore.lastPacketAt = now - 1000;
    session._lastAckOkAt = now - 100;
    t.equal(session._telemetryLossReason(now), null, 'a short gap is normal');
    session.telemetryStore.lastPacketAt = now - 1600;
    link.quality = 50;
    t.equal(session._telemetryLossReason(now), null, 'weak link: log packets just get lost, not a hang');
    t.end();
});

test('a stall of the shared dongle does not make the copters lost', t => {
    const {session, link} = makeFlyingSession();
    const now = Date.now();
    link.scheduler = {stalledForMs: () => 3000, lastStallEndAt: 0};
    session.telemetryStore.lastPacketAt = now - 4000;
    t.equal(session._telemetryLossReason(now), null, 'during the stall');
    link.scheduler = {stalledForMs: () => 0, lastStallEndAt: now - 200};
    t.equal(session._telemetryLossReason(now), null, 'right after it: silence counts from its end');
    t.ok(session.getFlightJournal().some(e => e.event === 'radioStall'), 'journal notes the stall');
    t.end();
});

test('lost in the air with the land delivered: shown as landing, flying only briefly', t => {
    const {session} = makeFlyingSession();
    session._handleLinkLost('telemetryTimeout');
    return wait(60).then(() => {
        t.ok(session.isLostInAirLanding(), 'landing, not hovering');
        t.ok(session._airborneAtLossUntil - Date.now() < 10000, 'not 90 s of «flying»');
        return settle(session);
    });
});

test('estimator reset before takeoff makes the start point the origin again', t => {
    const link = makeFakeLink();
    const session = new CrazyflieSession({link});
    session.telemetryStore.baselinePending = false;
    session.telemetryStore.baseline = {x: -0.69, y: 0.36};
    return session._resetEstimatorBeforeTakeoff(null).then(() => {
        t.same(session.telemetryStore.baseline, {x: 0, y: 0});
        t.end();
    });
});

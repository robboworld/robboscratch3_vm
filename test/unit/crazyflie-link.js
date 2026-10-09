const fs = require('fs');
const path = require('path');
const test = require('tap').test;
const babel = require('@babel/core');

const DCA_SRC = path.resolve(__dirname, '../../../RobboScratch3_DeviceControlAPI/src');
const moduleCache = {};

/**
 * Transpile a DCA ES module (and its relative imports) to CommonJS.
 * @param {string} filePath absolute path
 * @param {object} [stubs] absolute path (with .js) → exports used instead of the real module
 * @returns {object} module exports
 */
const loadDcaModule = function (filePath, stubs) {
    const resolved = filePath.endsWith('.js') ? filePath : `${filePath}.js`;
    if (stubs && stubs[resolved]) return stubs[resolved];
    if (moduleCache[resolved]) return moduleCache[resolved].exports;
    const source = fs.readFileSync(resolved, 'utf8');
    const {code} = babel.transformSync(source, {
        filename: resolved,
        presets: [['@babel/preset-env', {modules: 'commonjs', targets: {node: 'current'}}]]
    });
    const module = {exports: {}};
    moduleCache[resolved] = module;
    const localRequire = spec => (spec.startsWith('.') ?
        loadDcaModule(path.resolve(path.dirname(resolved), spec), stubs) :
        require(spec)); // eslint-disable-line global-require
    // eslint-disable-next-line no-new-func
    const fn = new Function('module', 'exports', 'require', code);
    fn(module, module.exports, localRequire);
    return module.exports;
};

const LinkStats = loadDcaModule(path.join(DCA_SRC, 'crazyflie/link-stats.js')).default;
const protocol = loadDcaModule(path.join(DCA_SRC, 'crazyflie/protocol.js'));
const {RadioScheduler} = loadDcaModule(path.join(DCA_SRC, 'crazyflie/radio-scheduler.js'));
const CrazyradioLink = loadDcaModule(path.join(DCA_SRC, 'crazyflie/crazyradio-link.js')).default;
const CrazyflieSession = loadDcaModule(path.join(DCA_SRC, 'crazyflie/session.js')).default;

/**
 * Adapter whose replies come from a function of the sent bytes.
 * @param {function(Array<number>): object} reply result for each packet
 * @returns {object} adapter stub with a log of sent packets
 */
const makeAdapter = function (reply) {
    const sent = [];
    const ok = (v, cb) => cb(true);
    let state = 'closed';
    return {
        sent,
        getState: () => state,
        open: cb => {
            state = 'opened';
            cb(true);
        },
        close: cb => {
            state = 'closed';
            cb();
        },
        setPower: ok,
        setArc: ok,
        setArdBytes: ok,
        setAckEnable: ok,
        setDatarate: ok,
        setChannel: ok,
        setAddress: ok,
        sendAndReceive: packet => {
            const bytes = Array.from(new Uint8Array(packet));
            sent.push(bytes);
            return Promise.resolve(Object.assign({state: true, retry: 0, data: new Uint8Array(0)}, reply(bytes)));
        }
    };
};

test('link stats: quality from ACK retries over the last packets', t => {
    const stats = new LinkStats();
    t.equal(stats.getQuality(), null);
    stats.record(true, 0);
    stats.record(true, 5);
    stats.record(false, 0);
    t.equal(stats.getQuality(), 50, '(10 + 5 + 0) / 3 * 10');
    t.end();
});

test('protocol: replies match with any link bits; param set by name packet', t => {
    t.ok(protocol.crtpHeaderIs(0x55, 0x51), 'LOG_CONTROL with safelink down bit');
    t.notOk(protocol.crtpHeaderIs(0x50, 0x51));
    const raw = protocol.buildParamSetByNamePacket('led.bitmask', protocol.PARAM_TYPE.UINT8, 0xff);
    const packet = Array.from(new Uint8Array(raw));
    const expected = [0x2f, 0x00].concat(Array.from(Buffer.from('led\0bitmask\0', 'ascii')), [0x08, 0xff]);
    t.same(packet, expected, 'port 2 / misc channel, group\\0name\\0, type, value');
    t.ok(protocol.isConsolePayload(new Uint8Array([0x04, 0x41])), 'console with link bit');
    t.end();
});

test('safelink: handshake, then alternating up bit and acknowledged down bit', t => {
    const adapter = makeAdapter(bytes => {
        if (bytes[0] === 0xff && bytes[1] === 0x05) return {data: new Uint8Array([0xff, 0x05, 0x01])};
        return {data: new Uint8Array([0x52 | 0x04, 1, 0, 0, 0])};
    });
    const link = new CrazyradioLink({scheduler: new RadioScheduler(adapter)});
    return link.open()
        .then(() => link.negotiateSafelink())
        .then(on => {
            t.ok(on, 'copter echoed the handshake');
            return link.sendPacket(new Uint8Array([0xff]).buffer);
        })
        .then(() => link.sendPacket(new Uint8Array([0xff]).buffer))
        .then(() => {
            const headers = adapter.sent.slice(1).map(p => p[0] & 0x0c);
            t.same(headers, [0x04, 0x08], 'up 0 / down 1, then up 1 / down 0');
            t.end();
        });
});

test('safelink: plain mode when the copter does not echo', t => {
    const adapter = makeAdapter(() => ({data: new Uint8Array(0)}));
    const link = new CrazyradioLink({scheduler: new RadioScheduler(adapter)});
    return link.open()
        .then(() => link.negotiateSafelink())
        .then(on => {
            t.notOk(on);
            return link.sendPacket(new Uint8Array([0x51, 1]).buffer);
        })
        .then(() => {
            t.equal(adapter.sent[adapter.sent.length - 1][0], 0x51, 'header untouched');
            t.end();
        });
});

test('session: firmware console lines are collected', t => {
    const session = new CrazyflieSession({link: {isOpen: () => false, getStats: () => ({})}});
    session.status.connected = true;
    session.status.state = 'connected';
    const text = s => new Uint8Array([0x00].concat(Array.from(Buffer.from(s, 'ascii'))));
    session._consumeRuntimePayload(text('SYS: low'));
    session._consumeRuntimePayload(text(' battery\nPM: ok\n'));
    t.same(session.getConsoleLines(), ['SYS: low battery', 'PM: ok']);
    t.end();
});

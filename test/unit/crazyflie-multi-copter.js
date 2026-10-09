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

const uri = loadDcaModule(path.join(DCA_SRC, 'crazyflie/radio-uri.js'));
const {RadioScheduler} = loadDcaModule(path.join(DCA_SRC, 'crazyflie/radio-scheduler.js'));
const eeprom = loadDcaModule(path.join(DCA_SRC, 'crazyflie/eeprom-config.js'));

/**
 * Crazyradio adapter stub: records control transfers and exchanges.
 * @param {Set<string>} [deadAddresses] hex addresses that never ACK
 * @returns {object} adapter
 */
const makeFakeAdapter = function (deadAddresses) {
    const log = [];
    let state = 'closed';
    let address = null;
    const control = (name, value) => cb => {
        log.push(`${name}:${value}`);
        cb(true);
    };
    return {
        log,
        getState: () => state,
        open: cb => {
            state = 'opened';
            log.push('open');
            cb(true);
        },
        close: cb => {
            state = 'closed';
            log.push('close');
            cb();
        },
        setPower: (v, cb) => control('power', v)(cb),
        setArc: (v, cb) => control('arc', v)(cb),
        setArdBytes: (v, cb) => control('ard', v)(cb),
        setAckEnable: (v, cb) => control('ack', v)(cb),
        setDatarate: (v, cb) => control('rate', v)(cb),
        setChannel: (v, cb) => control('channel', v)(cb),
        setAddress: (v, cb) => {
            address = uri.addressToHex(v);
            control('address', address)(cb);
        },
        sendAndReceive: () => {
            log.push(`send@${address}`);
            const dead = deadAddresses && deadAddresses.has(address);
            return Promise.resolve({state: !dead, data: new Uint8Array(0)});
        }
    };
};

const configFor = number => uri.radioConfigForCopter(1, number);

test('radio-uri: groups, numbers and URIs', t => {
    t.equal(uri.channelForGroup(1), 80, 'group 1 is the factory channel');
    t.equal(uri.channelForGroup(8), 122);
    t.equal(uri.groupForChannel(86), 2);
    t.equal(uri.addressToHex(uri.addressForCopterNumber(2)), 'E7E7E7E702');
    t.equal(uri.addressToHex(uri.addressForCopterNumber(null)), 'E7E7E7E7E7', 'factory address');
    t.equal(uri.copterNumberFromAddress(uri.addressForCopterNumber(15)), 15);
    t.equal(uri.copterNumberFromAddress(uri.addressForCopterNumber(null)), null);
    const built = uri.buildRadioUri(uri.radioConfigForCopter(2, 3));
    t.equal(built, 'radio://0/86/2M/E7E7E7E703');
    const parsed = uri.parseRadioUri(built);
    t.equal(parsed.channel, 86);
    t.equal(parsed.datarate, '2Mbps');
    t.equal(uri.addressToHex(parsed.address), 'E7E7E7E703');
    t.equal(uri.parseRadioUri('radio://0/200/2M'), null, 'channel out of range');
    t.equal(uri.radioConfigForCopter(3, null).channel, 80, 'new copters are found on the factory channel from any group');
    t.end();
});

test('scheduler: radio settings change only when the copter changes', t => {
    const adapter = makeFakeAdapter();
    const scheduler = new RadioScheduler(adapter);
    const holder = {};
    const send = number => scheduler.run(configFor(number), () => adapter.sendAndReceive(), {linkId: number});
    return scheduler.acquire(holder)
        .then(() => send(1))
        .then(() => send(1))
        .then(() => send(2))
        .then(() => {
            const addressSwitches = adapter.log.filter(e => e.startsWith('address:'));
            t.same(addressSwitches, ['address:E7E7E7E701', 'address:E7E7E7E702'], 'two switches for three packets');
            t.equal(adapter.log.filter(e => e.startsWith('channel:')).length, 1, 'channel set once');
            t.end();
        });
});

test('scheduler: copters take turns and urgent packets go first', t => {
    const adapter = makeFakeAdapter();
    const scheduler = new RadioScheduler(adapter);
    const order = [];
    const job = (number, urgent) => scheduler.run(configFor(number), () => {
        order.push(number);
        return Promise.resolve();
    }, {linkId: number, urgent: urgent});
    return scheduler.acquire({}).then(() => {
        const jobs = [job(1), job(1), job(1), job(2), job(2), job(3, true)];
        return Promise.all(jobs);
    })
        .then(() => {
            t.equal(order[1], 3, 'urgent right after the running job');
            t.same(order.slice(2), [2, 1, 2, 1], 'round-robin between copters');
            t.end();
        });
});

test('scheduler: a dead copter does not block the others', t => {
    const adapter = makeFakeAdapter(new Set(['E7E7E7E701']));
    const scheduler = new RadioScheduler(adapter);
    const send = number => scheduler.run(configFor(number), () => adapter.sendAndReceive(), {linkId: number});
    return scheduler.acquire({}).then(() => Promise.all([send(1), send(2), send(1), send(2)]))
        .then(results => {
            t.same(results.map(r => r.state), [false, true, false, true]);
            t.end();
        });
});

test('scheduler: dongle stays open until the last copter releases it', t => {
    const adapter = makeFakeAdapter();
    const scheduler = new RadioScheduler(adapter);
    const a = {};
    const b = {};
    return scheduler.acquire(a)
        .then(() => scheduler.acquire(b))
        .then(() => scheduler.release(a))
        .then(() => {
            t.ok(scheduler.isOpen(), 'still open for b');
            return scheduler.release(b);
        })
        .then(() => {
            t.notOk(scheduler.isOpen(), 'closed after the last release');
            t.equal(adapter.log.filter(e => e === 'open').length, 1, 'opened once');
            t.end();
        });
});

test('scheduler: flush rejects queued jobs of one link only', t => {
    const adapter = makeFakeAdapter();
    const scheduler = new RadioScheduler(adapter);
    let release;
    const gate = new Promise(resolve => {
        release = resolve;
    });
    return scheduler.acquire({}).then(() => {
        const running = scheduler.run(configFor(1), () => gate, {linkId: 1});
        const queued1 = scheduler.run(configFor(1), () => Promise.resolve('a'), {linkId: 1});
        const queued2 = scheduler.run(configFor(2), () => Promise.resolve('b'), {linkId: 2});
        scheduler.flush(1);
        release('done');
        return Promise.all([running, queued1.catch(e => e.code), queued2]);
    })
        .then(results => {
            t.same(results, ['done', 'linkQueueFlushed', 'b']);
            t.end();
        });
});

test('eeprom: v1 config round-trips with cflib layout', t => {
    const packed = eeprom.packEepromConfig({
        channel: 86,
        speed: 2,
        address: uri.addressForCopterNumber(4),
        pitchTrim: 0.5,
        rollTrim: -0.25
    });
    t.equal(packed.length, 21);
    t.same(Array.from(packed.slice(0, 5)), [0x30, 0x78, 0x42, 0x43, 1], 'magic "0xBC" + version 1');
    t.same(Array.from(packed.slice(15, 20)), [0xe7, 0x04, 0xe7, 0xe7, 0xe7], 'address upper byte + lower LE');
    const parsed = eeprom.parseEepromConfig(packed);
    t.equal(parsed.channel, 86);
    t.equal(parsed.speed, 2);
    t.equal(uri.addressToHex(parsed.address), 'E7E7E7E704');
    t.equal(parsed.pitchTrim, 0.5);
    t.equal(parsed.rollTrim, -0.25);
    const broken = packed.slice();
    broken[20] ^= 0xff;
    t.equal(eeprom.parseEepromConfig(broken), null, 'bad checksum rejected');
    t.end();
});

test('eeprom: memory port packets', t => {
    const read = new Uint8Array(eeprom.buildMemReadRequest(0, 0, 21));
    t.same(Array.from(read), [0x4d, 0, 0, 0, 0, 0, 21], 'port 4 / channel 1, <BIB');
    const reply = new Uint8Array([0x4d, 0, 0, 0, 0, 0, 0, 0x30, 0x78]);
    const parsed = eeprom.parseMemReadWriteResponse(reply, eeprom.MEM_CHAN_READ);
    t.equal(parsed.status, 0);
    t.same(Array.from(parsed.data), [0x30, 0x78]);
    const details = new Uint8Array([0x4c, 2, 0, 0, 0, 1, 0, 0]);
    t.same(eeprom.parseMemDetailsResponse(details), {id: 0, type: 0, size: 256});
    t.end();
});

/**
 * QCA with stubbed DeviceControlAPI / flasher and a fake dongle shared by every copter.
 * @param {Set<string>} presentAddresses hex addresses that ACK
 * @returns {{qca: object, adapter: object}} QCA instance and adapter
 */
const loadQcaWithFakeRadio = function (presentAddresses) {
    const adapter = makeFakeAdapter();
    adapter.sendAndReceive = () => Promise.resolve({
        state: presentAddresses.has(adapter.log.filter(e => e.startsWith('address:')).pop()
            .slice(8)),
        data: new Uint8Array(0)
    });
    const stubs = {
        [path.join(DCA_SRC, 'DeviceControlAPI.js')]: {__esModule: true, default: class {}},
        [path.join(DCA_SRC, 'crazyflie_flasher.js')]: {
            __esModule: true,
            isCrazyflieFlashingSupported: () => false,
            flashCrazyflieFirmwareZip: () => Promise.resolve(false),
            readCrazyflieFirmwareRevision: () => Promise.resolve({}),
            getBundledFirmwareRequiredRevision: () => null,
            getBundledFirmwareExpectedRevisionHex: () => null,
            resolveCf2FirmwareVersionLabel: () => null
        },
        [path.join(DCA_SRC, 'crazyradio.js')]: {__esModule: true, Crazyradio: adapter}
    };
    Object.keys(moduleCache).forEach(key => {
        if (key.indexOf(`${path.sep}crazyflie${path.sep}`) !== -1 || key.endsWith('QuadcopterControlAPI.js')) {
            delete moduleCache[key];
        }
    });
    const QuadcopterControlAPI = loadDcaModule(path.join(DCA_SRC, 'QuadcopterControlAPI.js'), stubs).default;
    return {qca: new QuadcopterControlAPI(), adapter};
};

test('QCA: copter handles are bound to their own address', t => {
    const {qca} = loadQcaWithFakeRadio(new Set());
    const two = qca.copter(2);
    t.equal(two.getCopterId(), 2);
    t.equal(qca.copter(2), two, 'handle cached');
    t.equal(uri.addressToHex(two.crazyflieSession.link.config.address), 'E7E7E7E702');
    t.equal(qca.getCopterId(), 'new', 'facade defaults to the factory copter');
    t.equal(qca.copter(99), null, 'invalid number');
    t.end();
});

test('QCA: scan finds the copters of the group', t => {
    const {qca} = loadQcaWithFakeRadio(new Set(['E7E7E7E701', 'E7E7E7E703', 'E7E7E7E7E7']));
    return qca._scanCopters().then(found => {
        t.same(found, [1, 3, 'new']);
        t.end();
    });
});

test('QCA: one lost ACK does not hide a copter from the scan', t => {
    const {qca, adapter} = loadQcaWithFakeRadio(new Set(['E7E7E7E702']));
    const answers = adapter.sendAndReceive;
    let firstProbe = true;
    adapter.sendAndReceive = (...args) => {
        const address = adapter.log.filter(e => e.startsWith('address:')).pop()
            .slice(8);
        if (address === 'E7E7E7E702' && firstProbe) {
            firstProbe = false;
            return Promise.resolve({state: false, data: new Uint8Array(0)});
        }
        return answers(...args);
    };
    return qca._scanCopters().then(found => {
        t.same(found, [2], 'found on the second probe');
        t.end();
    });
});

test('QCA: copter list, free numbers and the use-on-this-computer switch', t => {
    const {qca} = loadQcaWithFakeRadio(new Set());
    qca._lastScanIds = new Set([1, 'new']);
    qca._ensureSession(1);
    t.same(qca.getCopters().map(c => c.id), [1, 'new'], 'numbered first, factory last');
    t.notOk(qca.getFreeCopterNumbers().includes(1), 'number 1 is taken');
    return qca.setCopterEnabled(1, false).then(() => {
        t.equal(qca.isCopterEnabled(1), false);
        t.equal(qca.getCopters()[0].enabled, false);
        t.end();
    });
});

test('QCA: changing the group drops the copters of the old channel', t => {
    const {qca} = loadQcaWithFakeRadio(new Set());
    qca._ensureSession(2);
    return qca.setRadioGroup(3).then(changed => {
        t.ok(changed);
        t.equal(qca.getRadioGroup(), 3);
        t.notOk(qca.sessions.has(2));
        t.equal(qca.copter(2).crazyflieSession.link.config.channel, 92, 'numbered copters use the group channel');
        t.equal(qca.crazyflieSession.link.config.channel, 80, 'the factory copter stays on the factory channel');
        t.end();
    });
});

test('QCA: a connected factory copter gets the first free number automatically', t => {
    const {qca} = loadQcaWithFakeRadio(new Set());
    qca._lastScanIds = new Set([1, 'new']);
    qca._ensureSession(1);
    const factory = qca.sessions.get('new');
    factory.isConnected = () => true;
    factory.isAirborne = () => false;
    const calls = [];
    const factoryAddress = new Uint8Array([0xe7, 0xe7, 0xe7, 0xe7, 0xe7]);
    factory.readEepromConfig = () => Promise.resolve({channel: 80, address: factoryAddress});
    qca.assignCopterNumber = (from, number) => {
        calls.push([from, number]);
        return Promise.resolve(true);
    };
    qca._autoNumberArmed = true;
    qca._maybeAutoNumber();
    t.equal(calls.length, 0, 'not before its config memory was read');
    qca._autoNumberArmed = false;
    return qca._identifyFactoryCopter().then(() => {
        qca._maybeAutoNumber();
        t.equal(calls.length, 0, 'not without a search started by the user');
        qca._autoNumberArmed = true;
        qca._maybeAutoNumber();
        t.equal(qca.getCopters().find(c => c.id === 'new').numbering, true, 'palette sees the assignment');
        qca._maybeAutoNumber();
        return null;
    })
        .then(() => null)
        .then(() => {
            t.same(calls, [['new', 2]], 'one assignment, first free number');
            t.equal(qca._autoNumbering, false);
            t.same(qca._lastAutoNumber.number, 2);

            qca._autoNumberArmed = true;
            qca._autoNumberFailed = true;
            qca._maybeAutoNumber();
            t.equal(calls.length, 1, 'no retry after a failure until the next search');
            qca._autoNumberFailed = false;
            factory.isAirborne = () => true;
            qca._maybeAutoNumber();
            t.equal(calls.length, 1, 'not while flying');
            t.end();
        });
});

test('QCA: a numbered copter answering on the factory address is not numbered again', t => {
    // 09.10.2026: copter 1 answered as a factory copter for ~30 s after a crash.
    const {qca} = loadQcaWithFakeRadio(new Set());
    qca._lastScanIds = new Set(['new']);
    const factory = qca.sessions.get('new');
    factory.isConnected = () => true;
    factory.isAirborne = () => false;
    const ownAddress = qca.copter(1).crazyflieSession.link.config.address;
    factory.readEepromConfig = () => Promise.resolve({channel: 80, address: ownAddress});
    const calls = [];
    qca.assignCopterNumber = (from, number) => {
        calls.push([from, number]);
        return Promise.resolve(true);
    };
    qca._autoNumberArmed = true;
    return qca._identifyFactoryCopter().then(() => {
        qca._maybeAutoNumber();
        t.equal(calls.length, 0, 'no second number');
        t.equal(qca.getCopters().find(c => c.id === 'new').numberedAs, 1, 'the palette knows who it is');
        t.end();
    });
});

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

const battery = loadDcaModule(path.join(DCA_SRC, 'crazyflie/battery-policy.js'));
const BatteryPolicy = battery.default;

const P = battery.flightPercentFromRestVoltage;

/**
 * Feed samples every 500 ms.
 * @param {object} policy BatteryPolicy
 * @param {number} from start ms
 * @param {number} to end ms
 * @param {number} vbat voltage
 * @param {boolean} flying in the air
 * @returns {number} next timestamp
 */
const feed = function (policy, from, to, vbat, flying) {
    let t = from;
    for (; t <= to; t += 500) policy.update({vbat, flying}, t);
    return t;
};

test('battery: percent is the share of the flight left (3.75 V empty, 4.15 V full at rest)', t => {
    t.equal(P(3.75), 0, 'the firmware lands at 3.2 V loaded ≈ 3.75 V at rest');
    t.equal(P(4.15), 100);
    t.equal(P(4.3), 100);
    t.equal(Math.round(P(3.95)), 50);
    t.ok(battery.lipoPercentFromVoltage(3.68) < 10, 'old linear «60 %» is almost empty on the firmware curve too');
    t.end();
});

test('battery: percent comes from a 5 s moving average', t => {
    const policy = new BatteryPolicy();
    t.equal(policy.getState().level, 'unknown', 'no telemetry yet');
    policy.update({vbat: 4.0}, 0);
    policy.update({vbat: 3.0}, 100);
    t.equal(policy.getState().vbat, 3.5, 'spike averaged');
    policy.update({vbat: 3.95}, 6000);
    t.equal(policy.getState().vbat, 3.95, 'old samples leave the window');
    t.equal(policy.getState().percent, 50);
    t.end();
});

test('battery: warning below 3.2 V, critical after 5 s below 3.0 V or pm lowPower', t => {
    const policy = new BatteryPolicy();
    policy.update({vbat: 3.1}, 0);
    t.equal(policy.getState().level, 'warning');
    policy.update({vbat: 2.9}, 6000);
    t.equal(policy.getState().level, 'warning', 'short dip below 3.0 V is not critical yet');
    policy.update({vbat: 2.9}, 11500);
    t.equal(policy.getState().level, 'critical');
    const firmware = new BatteryPolicy();
    firmware.update({vbat: 3.9, pmState: battery.PM_STATE.LOW_POWER}, 0);
    t.equal(firmware.getState().level, 'critical', 'pm.state lowPower');
    t.end();
});

test('battery: capacity sets the flight time of a full battery', t => {
    t.ok(Math.abs(battery.fullFlightMinutesForCapacity(380) - 5) < 1e-9, '380 mAh ≈ 5 min (09.10.2026)');
    t.ok(Math.abs(battery.fullFlightMinutesForCapacity(300) - 3.95) < 0.01, 'stock 300 mAh ≈ 4 min');
    t.equal(new BatteryPolicy().getCapacity(), 300, 'stock battery by default');
    t.equal(battery.flightMinutesFromPercent(50, 5), 2.5);
    t.equal(battery.flightMinutesFromPercent(50), 3, 'simulator: 6 min a full battery');
    t.end();
});

test('battery: the flight to the landing on 09.10.2026 reaches 0 without jumps', t => {
    // 380 mAh, 3.985 V at rest (59 %), 182 s in the air to the low-voltage landing.
    const policy = new BatteryPolicy({capacityMah: 380});
    let now = feed(policy, 0, 5000, 3.985, false);
    const start = policy.getState().percent;
    t.equal(start, 59);
    let prev = start;
    let jumps = 0;
    for (let s = 0; s < 180; s++) {
        const loaded = s < 170 ? 3.43 - s * 0.0012 : 3.22 - (s - 170) * 0.003; // plateau, then the end
        policy.update({vbat: loaded, flying: true}, now);
        now += 1000;
        const p = policy.getState().percent;
        if (prev - p > 3) jumps += 1;
        prev = p;
    }
    t.equal(jumps, 0, 'no drops bigger than 3 % per second');
    t.ok(policy.getState().percent <= 3, `near the landing ${policy.getState().percent} %`);
    t.end();
});

test('battery: after landing the flight value stays until the voltage recovers above it', t => {
    const policy = new BatteryPolicy({capacityMah: 380});
    let now = feed(policy, 0, 5000, 4.15, false);
    t.equal(policy.getState().percent, 100);
    now = feed(policy, now, now + 60000, 3.5, true);
    const air = policy.getState().percent;
    t.ok(air >= 79 && air <= 81, `one minute of 5 takes 20 %: ${air} %`);
    now = feed(policy, now, now + 3000, 3.85, false);
    t.equal(policy.getState().percent, air, 'right after landing: the flight value, not the depressed 25 %');
    feed(policy, now, now + 60000, 4.12, false);
    t.equal(policy.getState().percent, Math.round(P(4.12)), 'the voltage recovered above it: the voltage again');
    t.end();
});

test('battery: a worn battery near 3.2 V loaded is pulled to 0', t => {
    // 07.10.2026: 47 % on the firmware curve on the ground, loaded 3.36 → 3.2 V in 30 s, then the landing.
    const policy = new BatteryPolicy();
    let now = feed(policy, 0, 5000, 3.91, false);
    now = feed(policy, now, now + 10000, 3.36, true);
    feed(policy, now, now + 20000, 3.2, true);
    t.equal(policy.getState().percent, 0);
    t.equal(policy.getState().minutes, 0);
    t.end();
});

test('battery: charging drops the kept flight value', t => {
    const policy = new BatteryPolicy();
    let now = feed(policy, 0, 5000, 4.1, false);
    now = feed(policy, now, now + 60000, 3.5, true);
    now = feed(policy, now, now + 3000, 3.8, false);
    t.ok(policy.getState().percent > 20, 'kept flight value');
    policy.update({vbat: 3.8, pmState: battery.PM_STATE.CHARGING, flying: false}, now);
    t.ok(Math.abs(policy.getState().percent - P(3.8)) <= 1, 'the voltage again');
    t.end();
});

test('battery: no takeoff when the charge is not enough for a flight', t => {
    // 09.10.2026: after a reboot 3.76 V at rest («~0 min») — the script took off again.
    const low = new BatteryPolicy();
    feed(low, 0, 5000, 3.76, false);
    t.equal(low.getState().canTakeOff, false, '3.76 V: refused');
    const ok = new BatteryPolicy();
    feed(ok, 0, 5000, 3.9, false);
    t.equal(ok.getState().canTakeOff, true, '3.9 V: allowed');
    const flying = new BatteryPolicy();
    const now = feed(flying, 0, 5000, 3.8, false);
    feed(flying, now, now + 30000, 3.4, true);
    t.equal(flying.getState().canTakeOff, true, 'in the air the flag does not apply (landing must stay possible)');
    t.end();
});

test('battery: a reconnect soon after a flight keeps the flight value', t => {
    // 09.10.2026: every lock in the air rebooted the copter; on reconnect the percent fell 86 → 72 %.
    const policy = new BatteryPolicy({capacityMah: 380});
    let now = feed(policy, 0, 5000, 4.12, false);
    now = feed(policy, now, now + 20000, 3.45, true);
    const air = policy.getState().percent;
    policy.reconnect(now + 5000);
    now = feed(policy, now + 5000, now + 10000, 3.95, false);
    t.equal(policy.getState().percent, air, 'lost in the air: the flight value, not the depressed 50 %');
    policy.reconnect(now + 200000);
    feed(policy, now + 200000, now + 205000, 3.9, false);
    t.equal(policy.getState().percent, Math.round(P(3.9)), 'minutes later: the voltage again');
    const swapped = new BatteryPolicy();
    now = feed(swapped, 0, 5000, 3.9, false);
    now = feed(swapped, now, now + 60000, 3.45, true);
    swapped.reconnect(now + 20000);
    feed(swapped, now + 20000, now + 25000, 4.15, false);
    t.equal(swapped.getState().percent, 100, 'a fuller battery shows its own percent');
    t.end();
});

test('battery: a glitch sample after a reconnect in the air does not zero the percent', t => {
    // 09.10.2026, copter 2: reconnected in the air, one bad sample, the percent fell 54 → 0 and stayed.
    const policy = new BatteryPolicy();
    let now = feed(policy, 0, 5000, 4.0, false);
    now = feed(policy, now, now + 30000, 3.5, true);
    const air = policy.getState().percent;
    policy.reconnect(now);
    policy.update({vbat: 0.8, flying: true}, now + 500);
    policy.update({vbat: 3.1, flying: true}, now + 1000);
    policy.update({vbat: 3.5, flying: true}, now + 1500);
    t.ok(policy.getState().percent >= air - 2, `kept ${policy.getState().percent} % (was ${air} %)`);
    t.end();
});

const tap = require('tap');
const meta = require('../../src/robbo/robbo-simulator-project-meta');

const test = tap.test;

test('buildRobboSimulatorMetaForSave normalizes five slots', t => {
    const m = meta.buildRobboSimulatorMetaForSave({
        simEnabled: true,
        extensionPackActivated: true,
        sensors: [
            {sensor_name: 'line', sensor_active: true, is_sensor_version_new: false}
        ]
    });
    t.equal(m.version, 1);
    t.equal(m.simEnabled, true);
    t.equal(m.extensionPackActivated, true);
    t.equal(m.sensors.length, 5);
    t.equal(m.sensors[0].sensor_name, 'line');
    t.equal(m.sensors[0].sensor_active, true);
    t.equal(m.sensors[1].sensor_name, 'nosensor');
    t.end();
});

test('parseRobboSimulatorMeta rejects wrong version', t => {
    t.equal(meta.parseRobboSimulatorMeta({version: 2, simEnabled: true, sensors: []}), null);
    t.end();
});

test('parseRobboSimulatorMeta normalizes invalid sensor names', t => {
    const p = meta.parseRobboSimulatorMeta({
        version: 1,
        simEnabled: false,
        extensionPackActivated: false,
        sensors: [
            {sensor_name: 'not-a-real-sensor', sensor_active: true, is_sensor_version_new: false}
        ]
    });
    t.ok(p);
    t.equal(p.sensors[0].sensor_name, 'nosensor');
    t.equal(p.sensors[0].sensor_active, false);
    t.end();
});

test('mergeRobboSimulatorMetaIntoProjectJson adds meta.robboSimulator', t => {
    const obj = {meta: {semver: '3.0.0'}, targets: []};
    const block = meta.buildRobboSimulatorMetaForSave({
        simEnabled: false,
        extensionPackActivated: false,
        sensors: []
    });
    meta.mergeRobboSimulatorMetaIntoProjectJson(obj, block);
    t.equal(obj.meta.semver, '3.0.0');
    t.equal(obj.meta.robboSimulator.version, 1);
    t.end();
});

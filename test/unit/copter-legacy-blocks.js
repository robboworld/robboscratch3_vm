const test = require('tap').test;
const sb3 = require('../../src/serialization/sb3');

const block = (opcode, parent) => ({
    opcode, parent, next: null, inputs: {}, fields: {}, shadow: false, topLevel: parent === null
});

test('old copter reporters load as [property] of copter', t => {
    const blocks = sb3.deserializeBlocks({
        a: block('copter_x_coord', 'c'),
        b: block('copter_battery', null),
        c: block('copter_change_x_by', null)
    });
    t.equal(blocks.a.opcode, 'copter_value');
    t.same(blocks.a.fields, {PROPERTY: {name: 'PROPERTY', value: 'x'}});
    t.equal(blocks.a.parent, 'c', 'stays in its place');
    t.equal(blocks.b.fields.PROPERTY.value, 'battery');
    t.equal(blocks.c.opcode, 'copter_change_x_by', 'an old command stays and shows its notice');
    t.end();
});

test('fly N meters / fly N seconds load as fly N [unit]', t => {
    const blocks = sb3.deserializeBlocks({
        d: Object.assign(block('copter_fly_distance', null), {
            inputs: {DIRECTION: [1, 'dir1'], METERS: [1, [4, '0.7']]}
        }),
        s: Object.assign(block('copter_fly_time', null), {
            inputs: {DIRECTION: [1, 'dir2'], SECONDS: [1, [4, '2']]}
        })
    });
    t.equal(blocks.d.opcode, 'copter_fly');
    t.equal(blocks.d.fields.UNIT.value, 'meters');
    t.notOk(blocks.d.inputs.METERS, 'old input name is gone');
    t.equal(blocks.d.inputs.AMOUNT.name, 'AMOUNT');
    t.equal(blocks[blocks.d.inputs.AMOUNT.block].fields.NUM.value, '0.7', 'the value stays');
    t.equal(blocks.d.inputs.DIRECTION.block, 'dir1', 'direction stays');
    t.equal(blocks.s.fields.UNIT.value, 'seconds');
    t.equal(blocks[blocks.s.inputs.AMOUNT.block].fields.NUM.value, '2');
    t.end();
});

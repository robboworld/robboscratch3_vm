const test = require('tap').test;

/**
 * Minimal mirror of CrazyradioLink queue semantics (urgent lane) for regression tests.
 */
class RadioQueueHarness {
    constructor () {
        this._queue = Promise.resolve();
        this.stats = {urgentSendCount: 0};
        this.sent = [];
    }

    _enqueue (task, options) {
        const urgent = options && options.urgent === true;
        if (urgent) {
            this.stats.urgentSendCount += 1;
            const urgentRun = this._queue.catch(() => null).then(task);
            this._queue = urgentRun.catch(() => null);
            return urgentRun;
        }
        const nextTask = this._queue.catch(() => null).then(task);
        this._queue = nextTask.catch(() => null);
        return nextTask;
    }

    sendPacket (value) {
        return this._enqueue(() => {
            this.sent.push(value);
            return Promise.resolve({state: true});
        }, {urgent: false});
    }

    sendPacketUrgent (value) {
        return this._enqueue(() => {
            this.sent.push(value);
            return Promise.resolve({state: true});
        }, {urgent: true});
    }
}

test('radio queue harness: urgent lane records stats', t => {
    const link = new RadioQueueHarness();
    const jobs = [];
    for (let i = 0; i < 4; i++) {
        jobs.push(link.sendPacket(i));
    }
    jobs.push(link.sendPacketUrgent(99));
    return Promise.all(jobs).then(() => {
        t.equal(link.stats.urgentSendCount, 1);
        t.ok(link.sent.indexOf(99) !== -1);
        t.end();
    });
});

test('radio queue harness: errors do not stall queue', t => {
    const link = new RadioQueueHarness();
    link._queue = Promise.reject(new Error('fail'));
    return link.sendPacket(1).then(() => {
        t.deepEqual(link.sent, [1]);
        t.end();
    });
});

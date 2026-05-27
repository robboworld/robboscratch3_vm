class QuadcopterCommandCoordinator {
    constructor (options) {
        this._activeCommand = null;
        this._onFlightCleanup = options && typeof options.onFlightCleanup === 'function'
            ? options.onFlightCleanup
            : null;
    }

    get activeCommand () {
        return this._activeCommand;
    }

    _getOwnerId (util) {
        if (util && util.thread && util.thread.topBlock) {
            return String(util.thread.topBlock);
        }
        return null;
    }

    cancel (reason) {
        if (!this._activeCommand) return;

        const current = this._activeCommand;
        this._activeCommand = null;

        if (current.cancel) {
            current.cancel(current.context, reason);
        }

        if (this._onFlightCleanup &&
            reason !== 'projectStopAll' &&
            reason !== 'manualStop') {
            this._onFlightCleanup(reason, current);
        }
    }

    runTargetCommand (key, util, handlers) {
        return this._runCommand('target', key, util, handlers);
    }

    runTimedCommand (key, util, handlers) {
        return this._runCommand('timed', key, util, handlers);
    }

    _runCommand (mode, key, util, handlers) {
        if (!handlers) return;

        const ownerId = this._getOwnerId(util);

        if (this._activeCommand) {
            if (this._activeCommand.ownerId !== ownerId) {
                util.yield();
                return;
            }
            if (this._activeCommand.key !== key || this._activeCommand.mode !== mode) {
                this.cancel('replaced');
            }
        }

        if (!this._activeCommand) {
            const context = handlers.start ? (handlers.start() || {}) : {};
            this._activeCommand = {
                key: key,
                mode: mode,
                ownerId: ownerId,
                context: context,
                startedAt: Date.now(),
                durationMs: handlers.durationMs || 0,
                timeoutMs: handlers.timeoutMs || 0,
                shouldFinish: handlers.shouldFinish,
                finish: handlers.finish,
                cancel: handlers.cancel
            };
            util.yield();
            return;
        }

        const command = this._activeCommand;
        const elapsedMs = Date.now() - command.startedAt;
        let shouldFinish = false;
        let timedOut = false;

        if (mode === 'timed') {
            shouldFinish = elapsedMs >= command.durationMs;
            if (!shouldFinish && typeof command.shouldFinish === 'function') {
                shouldFinish = command.shouldFinish(command.context, elapsedMs) === true;
            }
        } else if (typeof command.shouldFinish === 'function') {
            shouldFinish = command.shouldFinish(command.context, elapsedMs) === true;
        }

        if (!shouldFinish && command.timeoutMs > 0 && elapsedMs >= command.timeoutMs) {
            shouldFinish = true;
            timedOut = true;
        }

        if (!shouldFinish) {
            util.yield();
            return;
        }

        this._activeCommand = null;
        if (typeof command.finish === 'function') {
            command.finish(command.context, timedOut);
        }
    }
}

module.exports = QuadcopterCommandCoordinator;

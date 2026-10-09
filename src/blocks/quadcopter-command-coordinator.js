/**
 * One flight command at a time per copter. The command belongs to the script (thread) that started
 * it; other scripts wait — except a command with `preempt` (land), which interrupts it. A command
 * whose script is gone (stopped, restarted by a broadcast, clone deleted) is cancelled by the next
 * caller, and its timeout counts whoever calls.
 */
class QuadcopterCommandCoordinator {
    constructor (options) {
        this._activeCommand = null;
        this._onFlightCleanup = options && typeof options.onFlightCleanup === 'function' ?
            options.onFlightCleanup :
            null;
        this._isThreadAlive = options && typeof options.isThreadAlive === 'function' ?
            options.isThreadAlive :
            null;
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

        // projectStopAll / manualStop: the caller stops the copter itself. battery: the DCA lands it.
        // preempted: the land that interrupts takes over at once.
        if (this._onFlightCleanup &&
            reason !== 'projectStopAll' &&
            reason !== 'manualStop' &&
            reason !== 'battery' &&
            reason !== 'preempted') {
            this._onFlightCleanup(reason, current);
        }
    }

    /**
     * @param {string} key command key
     * @param {object} util block util of the calling script
     * @returns {boolean} this thread already runs the command (the block is re-entered while waiting)
     */
    isRunning (key, util) {
        return Boolean(this._activeCommand && this._activeCommand.key === key &&
            this._activeCommand.ownerId === this._getOwnerId(util) &&
            this._isOwnerAlive(this._activeCommand));
    }

    runTargetCommand (key, util, handlers) {
        return this._runCommand('target', key, util, handlers);
    }

    runTimedCommand (key, util, handlers) {
        return this._runCommand('timed', key, util, handlers);
    }

    /**
     * @param {object} command active command
     * @returns {boolean} its script is still running
     */
    _isOwnerAlive (command) {
        if (!command.thread || !this._isThreadAlive) return true;
        return this._isThreadAlive(command.thread);
    }

    /**
     * Finish / time out the active command from any caller.
     * @param {object} command active command
     * @returns {boolean} true when the command is still active afterwards
     */
    _advance (command) {
        const elapsedMs = Date.now() - command.startedAt;
        let shouldFinish = false;
        let timedOut = false;

        if (command.mode === 'timed') {
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
            return true;
        }

        this._activeCommand = null;
        if (typeof command.finish === 'function') {
            command.finish(command.context, timedOut);
        }
        return false;
    }

    _runCommand (mode, key, util, handlers) {
        if (!handlers) return;

        const ownerId = this._getOwnerId(util);
        const active = this._activeCommand;

        if (active && active.ownerId !== ownerId) {
            if (!this._isOwnerAlive(active)) {
                this.cancel('ownerGone');
            } else if (handlers.preempt === true) {
                this.cancel('preempted');
            } else if (this._advance(active) || this._activeCommand) {
                // Someone else's command: still finish or time it out, then wait our turn.
                util.yield();
                return;
            }
        } else if (active && (active.key !== key || active.mode !== mode || !this._isOwnerAlive(active))) {
            this.cancel(this._isOwnerAlive(active) ? 'replaced' : 'ownerGone');
        }

        if (!this._activeCommand) {
            const context = handlers.start ? (handlers.start() || {}) : {};
            this._activeCommand = {
                key: key,
                mode: mode,
                ownerId: ownerId,
                thread: util && util.thread ? util.thread : null,
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

        if (this._advance(this._activeCommand)) {
            util.yield();
        }
    }
}

module.exports = QuadcopterCommandCoordinator;

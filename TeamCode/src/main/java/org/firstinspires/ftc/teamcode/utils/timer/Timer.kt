package org.firstinspires.ftc.teamcode.utils.timer

import org.firstinspires.ftc.teamcode.utils.updateListener.IHandler
import org.firstinspires.ftc.teamcode.utils.updateListener.UpdateHandler

class Timer : IHandler {
    private val _timer = ElapsedTimeExtra()

    fun timePause() = _timer.pause()
    fun timeStart() = _timer.start()
    fun isTimePaused() = _timer.isPause()

    init {
        UpdateHandler.addHandler(this)
    }

    private enum class TimerState {
        DEFAULT_TIMER,
        SUPPLIER_TIMER,
        SUPPLIER_TIMEOUT
    }

    private var _state = TimerState.DEFAULT_TIMER

    private var _isActive = false

    private var _time = 0.0

    private var _supplier = { false }

    private var _action = {}
    private var _timeoutAction = {}

    fun start(time: Double, action: () -> Unit) {
        _time = time
        _action = action

        _state = TimerState.DEFAULT_TIMER
        _isActive = true
        _timer.reset()
    }

    fun start(suppler: () -> Boolean, action: () -> Unit) {
        _action = action
        _supplier = suppler

        _state = TimerState.SUPPLIER_TIMER
        _isActive = true
        _timer.reset()
    }

    fun start(suppler: () -> Boolean, action: () -> Unit, timeout: Double, timeoutAction: () -> Unit) {
        _action = action
        _supplier = suppler

        _time = timeout
        _timeoutAction = timeoutAction

        _state = TimerState.SUPPLIER_TIMEOUT
        _isActive = true
        _timer.reset()
    }

    override fun update() {
        if (!_isActive)
            return

        if (_state == TimerState.DEFAULT_TIMER) {
            if (_timer.seconds() > _time)
                stopAndRun()
        } else {
            if (!_supplier.invoke())
                stopAndRun()
            else if (_state == TimerState.SUPPLIER_TIMEOUT && _timer.seconds() > _time) {
                _timeoutAction.invoke()
                stop()
            }
        }
    }

    fun stopAndRun() {
        if(_isActive)
            _action.invoke()

        stop()
    }

    override fun stop() {
        _isActive = false
    }
}
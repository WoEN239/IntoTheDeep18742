package org.firstinspires.ftc.teamcode.utils.timer

import org.firstinspires.ftc.teamcode.utils.updateListener.IHandler
import org.firstinspires.ftc.teamcode.utils.updateListener.UpdateHandler

/**
 * Таймеры нужны для выполнения действия через определенны промежуток времяни
 *
 * Есть 3 режима работы которые запускаются тремя перегрузками функции start()
 *
 * start(time: Double, action: () -> Unit):
 * вызывает функцию action через time секунд
 *
 * start(suppler: () -> Boolean, action: () -> Unit):
 * вызывает функцию action когда suppler вернет false
 *
 * start(suppler: () -> Boolean, action: () -> Unit, timeout: Double, timeoutAction: () -> Unit):
 * вызывает функцию action когда suppler вернет false, но если action не будет исполнен спустя timeout секунд, то вызовится timeoutAction
 *
 * @author tikhonsmovzh
 * @see UpdateHandler
 */
class Timer {
    private val _timer = ElapsedTimeExtra()

    fun timePause() = _timer.pause()
    fun timeStart() = _timer.start()
    fun isTimePaused() = _timer.isPause()

    private enum class TimerType {
        DEFAULT_TIMER,
        SUPPLIER_TIMER,
        SUPPLIER_TIMEOUT,
        INACTIVE
    }

    private var _type = TimerType.INACTIVE

    private var _time = 0.0

    private var _supplier = { false }

    private var _action = {}
    private var _timeoutAction = {}

    fun start(time: Double, action: () -> Unit) {
        _time = time
        _action = action

        _type = TimerType.DEFAULT_TIMER
        _timer.reset()
    }

    fun start(suppler: () -> Boolean, action: () -> Unit) {
        _action = action
        _supplier = suppler

        _type = TimerType.SUPPLIER_TIMER
        _timer.reset()
    }

    fun start(
        suppler: () -> Boolean,
        action: () -> Unit,
        timeout: Double,
        timeoutAction: () -> Unit
    ) {
        _action = action
        _supplier = suppler

        _time = timeout
        _timeoutAction = timeoutAction

        _type = TimerType.SUPPLIER_TIMEOUT
        _timer.reset()
    }

    fun update() {
        when (_type) {
            TimerType.DEFAULT_TIMER -> {
                if (_timer.seconds() > _time)
                    stopAndRun()
            }

            TimerType.SUPPLIER_TIMER -> {
                if (!_supplier.invoke())
                    stopAndRun()
            }

            TimerType.SUPPLIER_TIMEOUT -> {
                if (_timer.seconds() > _time) {
                    _timeoutAction.invoke()
                    stop()
                }
            }

            else -> {
                return
            }
        }
    }

    val isActive: Boolean
        get() = _type != TimerType.INACTIVE

    fun stopAndRun() {
        stop()

        _action.invoke()
    }

    fun stop() {
        _type = TimerType.INACTIVE
    }
}
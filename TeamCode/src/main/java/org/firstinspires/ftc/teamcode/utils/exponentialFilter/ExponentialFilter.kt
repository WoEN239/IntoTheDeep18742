package org.firstinspires.ftc.teamcode.utils.exponentialFilter

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utils.updateListener.IHandler
import org.firstinspires.ftc.teamcode.utils.updateListener.UpdateHandler

/**
 * Класс экспоненциального фильтра. Нужен для оюъединения значений
 *
 * @author tikhonsmovzh
 */
class ExponentialFilter(var coef: Double) : IHandler {
    private val _deltaTime = ElapsedTime()

    init {
        UpdateHandler.addHandler(this)
    }

    override fun start() {
        _deltaTime.reset()
    }

    fun updateRaw(value: Double, delta: Double): Double {
        val result = value + delta * (_deltaTime.seconds() / (coef + _deltaTime.seconds()))

        _deltaTime.reset()

        return result
    }

    fun update(val1: Double, val2: Double): Double {
        return updateRaw(val1, val2 - val1)
    }
}
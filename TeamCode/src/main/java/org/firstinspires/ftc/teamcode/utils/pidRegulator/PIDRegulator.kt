package org.firstinspires.ftc.teamcode.utils.pidRegulator

import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utils.updateListener.IHandler
import org.firstinspires.ftc.teamcode.utils.updateListener.UpdateHandler
import kotlin.math.abs
import kotlin.math.sign

/**
 * Класс для настроек пида, класс можно использовать в дашборде
 *
 * @author tikhonsmovzh
 * @see PIDRegulator
 */
data class PIDConfig(
    @JvmField var p: Double,
    @JvmField var i: Double = 0.0,
    @JvmField var limitI: Double = 0.0,
    @JvmField var d: Double = 0.0,
    @JvmField var f: Double = 0.0,
    @JvmField var g: Double = 0.0,
    @JvmField var limitU: Double = -1.0,
    @JvmField var fr: Double = 0.0,
    @JvmField var resetZeroIntegral: Boolean = false
)

/**
 * Класс пид регулятора с f и g дополнениями
 *
 * @author tikhonsmovzh
 * @see UpdateHandler
 * @see PIDConfig
 */
class PIDRegulator(var config: PIDConfig) : IHandler {
    private val _deltaTime = ElapsedTime()
    private var _integral = 0.0
    private var _errOld = 0.0

    init {
        UpdateHandler.addHandler(this)
    }

    fun update(err: Double, target: Double = 0.0): Double {
        val uP = err * config.p

        _integral += err * _deltaTime.seconds()
        _integral = clamp(_integral, -config.limitI / config.i, config.limitI / config.i)

        if(abs(sign(err) - sign(_errOld)) > 0.01 && config.resetZeroIntegral)
            resetIntegral()

        val uI = _integral * config.i

        val uD = (err - _errOld) / _deltaTime.seconds() * config.d
        _errOld = err

        var u = uP + uI + uD + target * config.f + config.g + sign(target) * config.fr

        if(config.limitU > 0.0)
            u = clamp(u, -config.limitU, config.limitU)

        _deltaTime.reset()

        return u
    }

    fun resetIntegral(){
        _integral = 0.0
    }

    override fun start() {
        _deltaTime.reset()
    }
}
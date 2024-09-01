package org.firstinspires.ftc.teamcode.utils.pidRegulator

import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utils.updateListener.IHandler
import org.firstinspires.ftc.teamcode.utils.updateListener.UpdateHandler

data class PIDConfig(
    var p: Double,
    var i: Double = 0.0,
    var limitI: Double = 0.0,
    var d: Double = 0.0,
    var f: Double = 0.0,
    var g: Double = 0.0,
    var limitU: Double = 1.0
)

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
        _integral = clamp(_integral, -config.limitI, config.limitI)
        val uI = _integral * config.i

        val uD = (err - _errOld) / _deltaTime.seconds() * config.d
        _errOld = err

        var u = uP + uI + uD + target * config.f + config.g
        u = clamp(u, -config.limitU, config.limitU)

        _deltaTime.reset()

        return u
    }

    override fun start() {
        _deltaTime.reset()
    }
}
package org.firstinspires.ftc.teamcode.utils.motor

import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.pidRegulator.PIDRegulator
import org.firstinspires.ftc.teamcode.utils.updateListener.IHandler
import org.firstinspires.ftc.teamcode.utils.updateListener.UpdateHandler

class Motor(val motor: DcMotorEx): IHandler {
    private val _maxVelocityTicks = 2400

    init {
        UpdateHandler.addHandler(this)
    }

    private val _velocityPid = PIDRegulator(Configs.MotorConfig.VELOCITY_PID)
    val encoder = EncoderController(motor)

    var targetTicksVelocity = 0.0

    var targetPower
        get() = targetTicksVelocity / _maxVelocityTicks
        set(value) {
            targetTicksVelocity = value * _maxVelocityTicks
        }

    override fun update() {
        val u = _velocityPid.update(encoder.velocity - targetTicksVelocity, targetTicksVelocity)

        motor.power = u
    }
}
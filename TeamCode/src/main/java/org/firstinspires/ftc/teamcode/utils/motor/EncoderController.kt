package org.firstinspires.ftc.teamcode.utils.motor

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.updateListener.IHandler
import org.firstinspires.ftc.teamcode.utils.updateListener.UpdateHandler

class EncoderController(val encoder: DcMotorEx) : IHandler {
    private var _oldPosition = 0

    var velocity: Double = 0.0

    val position
        get() = encoder.currentPosition

    init{
        UpdateHandler.addHandler(this)
    }

    private val _deltaTime = ElapsedTime()

    override fun update() {
        val hardwareSpeed: Double = encoder.velocity

        if (_deltaTime.seconds() > 0.085) {
            val mathSpeed = (position - _oldPosition) / _deltaTime.seconds()
            _deltaTime.reset()
            _oldPosition = position

            velocity = hardwareSpeed + Math.round((mathSpeed - hardwareSpeed) / (1 shl 16).toDouble()) * (1 shl 16).toDouble()
        }
    }

    override fun start() {
        encoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        encoder.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        _deltaTime.reset()
        _oldPosition = position
    }
}
package org.firstinspires.ftc.teamcode.utils.motor

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utils.updateListener.IHandler
import org.firstinspires.ftc.teamcode.utils.updateListener.UpdateHandler

class EncoderController(val encoder: DcMotorEx) : IHandler {
    private var _oldPosition = 0

    val velocity: Double = 0.0
        get(){
            val hardwareSpeed: Double = encoder.velocity

            if (_deltaTime.seconds() > 0.085) {
                val mathSpeed = (position - _oldPosition) / _deltaTime.seconds()
                _deltaTime.reset()
                _oldPosition = position

                return hardwareSpeed + Math.round((mathSpeed - hardwareSpeed) / (1 shl 16).toDouble()) * (1 shl 16).toDouble()
            }
            else
                return field
        }

    val position
        get() = encoder.currentPosition

    init{
        UpdateHandler.addHandler(this)
    }

    private val _deltaTime = ElapsedTime()

    override fun start() {
        _deltaTime.reset()
    }
}
package org.firstinspires.ftc.teamcode.utils.motor

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utils.updateListener.IHandler
import org.firstinspires.ftc.teamcode.utils.updateListener.UpdateHandler

/**
 * Класс позволяющий исправить проблему скорости в реве.
 * И автовыщитывание оборотов
 *
 * @see EncoderOnly
 * @see Motor
 * @see MotorOnly
 *
 * @author tikhonsmovzh
 */
class EncoderFix(val encoder: DcMotorEx, val calculateTurn: (Int) -> Double) : IHandler {
    private var _oldPosition = 0

    var velocity: Double = 0.0

    val position
        get() = encoder.currentPosition

    val calcPos
        get() = calculateTurn(position)

    init{
        UpdateHandler.addHandler(this)

        if(encoder !is EncoderOnly) {
            encoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            encoder.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
    }

    private val _deltaTime = ElapsedTime()

    private var _lastMathSpeed = 0.0

    override fun update() {
        val hardwareSpeed: Double = encoder.velocity

        if (_deltaTime.seconds() > 0.045) {
            _lastMathSpeed = (position - _oldPosition.toDouble()) / _deltaTime.seconds()
            _deltaTime.reset()
            _oldPosition = position
        }

        velocity = hardwareSpeed + Math.round((_lastMathSpeed - hardwareSpeed) / (1 shl 16).toDouble()) * (1 shl 16).toDouble()
    }

    override fun start() {
        _deltaTime.reset()
        _oldPosition = position
    }
}
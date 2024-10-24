package org.firstinspires.ftc.teamcode.utils.softServo

import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.devices.Battery
import org.firstinspires.ftc.teamcode.utils.servoAngle.ServoAngle
import org.firstinspires.ftc.teamcode.utils.updateListener.IHandler
import org.firstinspires.ftc.teamcode.utils.updateListener.UpdateHandler
import java.lang.Math.pow
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign
import kotlin.math.sqrt

/**
 * Класс для кправления сервоприводом с учетом ускорения и максимальной скоростью
 *
 * @see UpdateHandler
 * @author tikhonsmovzh
 */
class SoftServo(
    val servo: Servo,
    private val _startPosition: Double = 0.0,
    var E: Double = Configs.SoftServo.DEFAULT_E,
    var WMax: Double = Configs.SoftServo.DEFAULT_W_MAX
) : IHandler {
    private val _servoTime = ElapsedTime()

    private var t2 = 0.0
    private var t3 = 0.0
    private var t4 = 0.0
    private var t5 = 0.0
    private var yAbs = 0.0
    private var sign = 0.0
    private var t2Pow = 0.0
    private var y0 = 0.0

    var targetAngle: Double
        set(value) {
            if(servo !is ServoAngle)
                throw Exception("Default servo not support angle, request ServoAngle")

            targetPosition = value / servo.maxAngle
        }
        get() {
            if(servo !is ServoAngle)
                throw Exception("Default servo not support angle, request ServoAngle")

            return targetPosition * servo.maxAngle
        }

    var targetPosition: Double = -1.0
        set(value) {
            if(value < 0)
                return

            if (abs(value - field) < 0.002) {
                return
            }

            y0 = currentPosition

            _servoTime.reset()

            yAbs = abs(currentPosition - value)
            sign = sign(value - currentPosition)

            t2 = WMax / E
            t3 = yAbs / WMax - WMax / E + t2

            if (t3 > t2)
                t2Pow = E * t2.pow(2) / 2
            else {
                t4 = sqrt(yAbs / E)

                t5 = t4 * 2
            }

            field = value
        }

    var currentPosition
        get() = servo.position
        private set(value) {
            servo.position = value
        }

    val currentAngle: Double
        get(){
            if(servo !is ServoAngle)
                throw Exception("Default servo not support angle, request ServoAngle")

            return servo.angle
        }

    init {
        UpdateHandler.addHandler(this)
    }

    override fun start() {
        currentPosition = _startPosition
        targetPosition = _startPosition
    }

    override fun update() {
        if (t3 > t2) {
            if (_servoTime.seconds() <= t2 + t3) {
                if (_servoTime.seconds() <= t2)
                    currentPosition = y0 + sign * (E * _servoTime.seconds().pow(2) / 2)
                else if (_servoTime.seconds() <= t3)
                    currentPosition = y0 + sign * (t2Pow + WMax * (_servoTime.seconds() - t2))
                else
                    currentPosition =
                        y0 + sign * (t2Pow + WMax * (t3 - t2) + WMax * (_servoTime.seconds() - t3) - E * (_servoTime.seconds() - t3).pow(2) / 2)
            }

            return
        }

        if (_servoTime.seconds() <= t5) {
            if (_servoTime.seconds() <= t4)
                currentPosition = y0 + sign * (E * _servoTime.seconds().pow(2) / 2)
            else
                currentPosition = y0 + sign * (E * t4.pow(2) / 2 + sqrt(yAbs / E) * E * (_servoTime.seconds() - t4) - E * (_servoTime.seconds() - t4).pow(
                    2
                ) / 2)
        }
    }

    val isEnd get() = _servoTime.seconds() > t5 || (t3 > t2 && _servoTime.seconds() > t2 + t3)
}
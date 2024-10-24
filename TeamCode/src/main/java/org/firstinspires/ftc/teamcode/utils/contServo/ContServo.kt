package org.firstinspires.ftc.teamcode.utils.contServo

import com.qualcomm.robotcore.hardware.PwmControl.PwmRange
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.updateListener.IHandler
import org.firstinspires.ftc.teamcode.utils.updateListener.UpdateHandler
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign
import kotlin.math.sqrt


class ContServo(
    private val _servo: Servo,
    var E: Double = Configs.ContServo.DEFAULT_E,
    var maxRadSpeed: Double = Configs.ContServo.DEFAULT_MAX_VELOCITY
) : IHandler {
    init {
        UpdateHandler.addHandler(this)
    }

    private val _servoTime = ElapsedTime()

    private var t2 = 0.0
    private var t3 = 0.0
    private var t4 = 0.0
    private var t5 = 0.0
    private var sign = 0.0
    private var y0 = 0.0
    private var t2Pow = 0.0
    private var yAbs = 0.0

    var currentPosition: Double = 0.0
        private set

    fun resetAngleTo(ang: Double){
        currentPosition = ang
        _targetAngle = ang
    }

    fun resetAngle() = resetAngleTo(0.0)

    private var _targetAngle = 0.0

    var targetAngle: Double
        get() = _targetAngle
        set(value) {
            if (abs(value - _targetAngle) < 0.002) {
                return
            }

            _servoTime.reset()

            y0 = currentPosition

            yAbs = abs(currentPosition - value)
            sign = sign(value - currentPosition)

            t2 = maxRadSpeed / E
            t3 = yAbs / maxRadSpeed - maxRadSpeed / E + t2

            if (t3 > t2)
                t2Pow = E * t2.pow(2) / 2
            else {
                t4 = sqrt(yAbs / E)

                t5 = t4 * 2
            }

            _targetAngle = value
        }

    var currentVelocity: Double = 0.0
        private set(value){
            _servo.position = value / maxRadSpeed + 0.5

            field = value
        }

    override fun update() {
        if (t3 > t2) {
            if (_servoTime.seconds() <= t2 + t3) {
                if (_servoTime.seconds() <= t2) {
                    currentVelocity = sign * E * _servoTime.seconds()

                    currentPosition = y0 + sign * (E * _servoTime.seconds().pow(2) / 2)
                }
                else if (_servoTime.seconds() <= t3) {
                    currentVelocity = maxRadSpeed * sign

                    currentPosition = y0 + sign * (t2Pow + maxRadSpeed * (_servoTime.seconds() - t2))
                }
                else {
                    currentVelocity = sign * (maxRadSpeed - (_servoTime.seconds() - t3) * E)

                    currentPosition =
                        y0 + sign * (t2Pow + maxRadSpeed * (t3 - t2) + maxRadSpeed * (_servoTime.seconds() - t3) - E * (_servoTime.seconds() - t3).pow(2) / 2)
                }
            }
            else
                currentVelocity = 0.0

            return
        }

        if (_servoTime.seconds() <= t5) {
            if (_servoTime.seconds() <= t4) {
                currentVelocity = sign * E * _servoTime.seconds()

                currentPosition = y0 + sign * (E * _servoTime.seconds().pow(2) / 2)
            }
            else {
                currentVelocity = sign * (t4 * E - E * (_servoTime.seconds() - t4))

                currentPosition = y0 + sign * (E * t4.pow(2) / 2 + sqrt(yAbs / E) * E * (_servoTime.seconds() - t4) - E * (_servoTime.seconds() - t4).pow(2) / 2)
            }
        }
        else
            currentVelocity = 0.0
    }

    val isEnd get() = _servoTime.seconds() > t5 || (t3 > t2 && _servoTime.seconds() > t2 + t3)
}
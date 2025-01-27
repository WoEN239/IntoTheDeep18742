package org.firstinspires.ftc.teamcode.modules.intake

import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.utils.LEDLine.LEDLine
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.softServo.SoftServo

class Intake{
    private lateinit var _servoClamp: SoftServo

    private lateinit var _servoDifLeft: SoftServo
    private lateinit var _servoDifRight: SoftServo

    private lateinit var _leftLED: LEDLine
    private lateinit var _rightLED: LEDLine

    fun atTarget() = _servoClamp.isEnd && _servoDifLeft.isEnd && _servoDifRight.isEnd

    var xVelocity = 0.0
    var yVelocity = 0.0

    var xPos = 0.0
    var yPos = 0.0
    private val _deltaTime = ElapsedTime()

    fun init(collector: BaseCollector) {
        _servoClamp = SoftServo(collector.devices.servoClamp, Configs.IntakeConfig.SERVO_CLAMP)

        _servoDifLeft = SoftServo(collector.devices.servoDifLeft, 0.5)
        _servoDifRight = SoftServo(collector.devices.servoDifRight, 0.5)

        collector.devices.servoDifLeft.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        collector.devices.servoDifRight.pwmRange = PwmControl.PwmRange(500.0, 2500.0)

        _leftLED = collector.devices.leftLight
        _rightLED = collector.devices.rightLight
    }

    var clamp = ClampPosition.SERVO_CLAMP
        set(value) {
            if (value == ClampPosition.SERVO_CLAMP) {
                _servoClamp.targetPosition = Configs.IntakeConfig.SERVO_CLAMP

                _leftLED.power = Configs.Lighting.ON_POWER
                _rightLED.power = Configs.Lighting.ON_POWER
            }
            else {
                _servoClamp.targetPosition = Configs.IntakeConfig.SERVO_UNCLAMP

                _leftLED.power = Configs.Lighting.OFF_POWER
                _rightLED.power = Configs.Lighting.OFF_POWER
            }

            field = value
        }

    fun setDifPos(xRot: Double, yRot: Double)
    {
        xPos = xRot
        yPos = yRot

        val x = xRot + Configs.IntakeConfig.DIF_DIFFERENCE_X
        val y = (yRot + Configs.IntakeConfig.DIF_DIFFERENCE_Y) * Configs.IntakeConfig.GEAR_RATIO

        _servoDifRight.targetPosition = clamp((y + x) / Configs.IntakeConfig.MAX, 0.0, 1.0)
        _servoDifLeft.targetPosition = clamp(1.0 - (x - y) / Configs.IntakeConfig.MAX, 0.0, 1.0)
    }

    enum class ClampPosition
    {
        SERVO_CLAMP,
        SERVO_UNCLAMP
    }

    fun update() {
        setDifPos(clamp(xPos + _deltaTime.seconds() * xVelocity, -90.0, 90.0), clamp(yPos + _deltaTime.seconds() * yVelocity, -180.0, 180.0))

        _deltaTime.reset()
    }

    fun start() {
        _deltaTime.reset()
    }
}
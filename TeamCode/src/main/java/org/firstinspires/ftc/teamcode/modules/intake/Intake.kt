package org.firstinspires.ftc.teamcode.modules.intake

import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.utils.configs.Configs

class Intake{
    private lateinit var _servoClamp: Servo

    private lateinit var _servoDifLeft: ServoImplEx
    private lateinit var _servoDifRight: ServoImplEx

    var xVelocity = 0.0
    var yVelocity = 0.0

    var xPos = 0.0
    var yPos = 0.0
    private val _deltaTime = ElapsedTime()

    fun init(collector: BaseCollector) {
        _servoClamp = collector.devices.servoClamp

        _servoDifLeft = collector.devices.servoDifLeft
        _servoDifRight = collector.devices.servoDifRight

        _servoDifLeft.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        _servoDifRight.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
    }

    var clamp = ClampPosition.SERVO_UNCLAMP
        set(value) {
            if (value == ClampPosition.SERVO_CLAMP)
                _servoClamp.position = Configs.IntakeConfig.SERVO_CLAMP
            else
                _servoClamp.position = Configs.IntakeConfig.SERVO_UNCLAMP

            field = value
        }


    fun setDifPos(xRot: Double, yRot: Double)
    {
        xPos = xRot
        yPos = yRot

        val x = xRot + 135.0
        val y = yRot + 10.0

        _servoDifRight.position = clamp((y + x) / Configs.IntakeConfig.MAX, 0.0, 1.0)
        _servoDifLeft.position = clamp(1.0 - (x - y) / Configs.IntakeConfig.MAX, 0.0, 1.0)
    }

    enum class ClampPosition
    {
        SERVO_CLAMP,
        SERVO_UNCLAMP
    }

    fun update() {
        setDifPos(clamp(xPos + _deltaTime.seconds() * xVelocity, -90.0, 90.0), clamp(yPos + _deltaTime.seconds() * yVelocity, -90.0, 90.0))

        _deltaTime.reset()
    }

    fun start() {
        _deltaTime.reset()
        setDifPos(-90.0, 0.0)
        clamp = ClampPosition.SERVO_CLAMP
    }
}
package org.firstinspires.ftc.teamcode.modules.intake

import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.servoAngle.ServoAngle
import org.firstinspires.ftc.teamcode.utils.softServo.SoftServo

object Intake : IRobotModule {
    private lateinit var _horizontalServoLeft: SoftServo
    private lateinit var _horizontalServoRight: SoftServo

    private lateinit var _servoClamp: Servo
    private lateinit var _servoRotate: Servo
    private lateinit var _servoClampForv: Servo

    private lateinit var _servoFlip: ServoImplEx

    private lateinit var _endingFlipped: DigitalChannel
    private lateinit var _endingUnflipped: DigitalChannel

    override fun init(collector: BaseCollector) {
        _horizontalServoLeft = SoftServo(collector.devices.horizontalServoLeft, 0.1)
        _horizontalServoRight = SoftServo(collector.devices.horizontalServoRight, 0.88)

        _servoClamp = collector.devices.servoClamp
        _servoRotate = collector.devices.servoRotate
        _servoClampForv = collector.devices.servoClampForv

        _servoFlip = collector.devices.servoFlip
        _servoFlip.pwmRange = PwmRange(500.0, 2500.0)

        _endingFlipped = collector.devices.endingFlipped
        _endingUnflipped = collector.devices.endingUnflipped
    }

    var flip = GalaxyFlipPosition.SERVO_FLIP

    var clamp = ClampPosition.SERVO_UNCLAMP
        set(value) {
            if (value == ClampPosition.SERVO_CLAMP) {
                _servoClamp.position = Configs.IntakeConfig.SERVO_CLAMP
            } else if (value == ClampPosition.SERVO_UNCLAMP) {
                _servoClamp.position = Configs.IntakeConfig.SERVO_UNCLAMP
            }

            field = value
        }

    var position = AdvancedPosition.SERVO_UNPROMOTED
        set(value) {
            if (value == AdvancedPosition.SERVO_PROMOTED) {
                _horizontalServoLeft.targetPosition = Configs.IntakeConfig.SERVO_PROMOTED_LEFT
                _horizontalServoRight.targetPosition = Configs.IntakeConfig.SERVO_PROMOTED_RIGHT
            } else if (value == AdvancedPosition.SERVO_UNPROMOTED) {
                _horizontalServoLeft.targetPosition = Configs.IntakeConfig.SERVO_UNPROMOTED_LEFT
                _horizontalServoRight.targetPosition = Configs.IntakeConfig.SERVO_UNPROMOTED_RIGHT
            }

            field = value
        }

    var clampF = ClampPositionF.SERVO_UNCLAMPF
        set(value) {
            if (value == ClampPositionF.SERVO_CLAMPF) {
                _servoClampForv.position = Configs.IntakeConfig.SERVO_CLAMPF
        } else if (value == ClampPositionF.SERVO_UNCLAMPF) {
                _servoClampForv.position = Configs.IntakeConfig.SERVO_UNCLAMPF
            }

            field = value
        }

    val servoRotatePosition
        get() = _servoRotate.position

    var servoRotateVelocity: Double = 0.0

    private val _deltaTime = ElapsedTime()

    override fun start() {
        _deltaTime.reset()
    }

    override fun update() {
        if (flip == GalaxyFlipPosition.SERVO_FLIP) {
            if (!_endingFlipped.state)
                _servoFlip.position =
                    Configs.IntakeConfig.FLIP_STOP_POSITION + Configs.IntakeConfig.FLIP_VELOCITY
            else
                _servoFlip.position = Configs.IntakeConfig.FLIP_STOP_POSITION
        } else {
            if (!_endingUnflipped.state)
                _servoFlip.position =
                    Configs.IntakeConfig.FLIP_STOP_POSITION - Configs.IntakeConfig.FLIP_VELOCITY
            else
                _servoFlip.position = Configs.IntakeConfig.FLIP_STOP_POSITION
        }

        if(position == AdvancedPosition.SERVO_PROMOTED)
        _servoRotate.position = clamp(_servoRotate.position + servoRotateVelocity * _deltaTime.seconds(), 0.0, 1.0)
          else {
            servoRotateVelocity = 0.0
            _servoRotate.position = 0.5
        }
        _deltaTime.reset()
    }

    enum class AdvancedPosition//нижняя
    {
        SERVO_PROMOTED,
        SERVO_UNPROMOTED
    }

    enum class ClampPosition// захват
    {
        SERVO_CLAMP,
        SERVO_UNCLAMP
    }
    enum class ClampPositionF// захват
    {
        SERVO_CLAMPF,
        SERVO_UNCLAMPF
    }

    enum class GalaxyFlipPosition {
        SERVO_UNFLIP,
        SERVO_FLIP
    }
}
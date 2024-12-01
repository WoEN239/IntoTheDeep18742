package org.firstinspires.ftc.teamcode.modules.intake

import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.modules.lift.Lift.SetLiftTargetEvent
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.servoAngle.ServoAngle
import org.firstinspires.ftc.teamcode.utils.softServo.SoftServo
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry

class Intake() : IRobotModule {
    private lateinit var _servoClamp: Servo
    private lateinit var _servoDifleft: Servo
    private lateinit var _servoDifRight: Servo
    private var _liftTarget = 0.0

    override fun init(collector: BaseCollector, bus: EventBus) {
        _servoClamp = collector.devices.servoClamp
        _servoDifleft = collector.devices.servoDifLeft
        _servoDifRight = collector.devices.servoDifRight

        bus.subscribe(SetLiftTargetEvent::class){
            _liftTarget = it.targetAimPos / 2400.0 * 360

            StaticTelemetry.addLine("liftAngle = " + _liftTarget)
        }
    }

    /*var flip = GalaxyFlipPosition.SERVO_FLIP

    var clamp = ClampPosition.SERVO_UNCLAMP
        set(value) {
            if (value == ClampPosition.SERVO_CLAMP) {
                _servoClamp.position = Configs.IntakeConfig.SERVO_CLAMP
            } else if (value == ClampPosition.SERVO_UNCLAMP) {
                _servoClamp.position = Configs.IntakeConfig.SERVO_UNCLAMP
            }

            field = value
        }
*/


    

    /*var rotateUp = RotatePositionUp.SERVO_UNROTATEUP
        set(value) {
            if (value == RotatePositionUp.SERVO_ROTATEUP) {
                _servoRotateUp.position = Configs.IntakeConfig.SERVO_ROTATEUP
            } else if (value == RotatePositionUp.SERVO_UNROTATEUP) {
                _servoRotateUp.position = Configs.IntakeConfig.SERVO_UNROTATEUP
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

    var clampUp = ClampPositionUp.SERVO_UNCLAMPUP
        set(value) {
            if (value == ClampPositionUp.SERVO_CLAMPUP) {
                _servoClampUp.position = Configs.IntakeConfig.SERVO_CLAMPUP
            } else if (value == ClampPositionUp.SERVO_UNCLAMPUP) {
                _servoClampUp.position = Configs.IntakeConfig.SERVO_UNCLAMPUP
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
        */

   // }

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
    enum class ClampPositionUp// захват
    {
        SERVO_CLAMPUP,
        SERVO_UNCLAMPUP
    }

    enum class GalaxyFlipPosition {
        SERVO_UNFLIP,
        SERVO_FLIP
    }
    enum class RotatePositionUp
    {
        SERVO_ROTATEUP,
        SERVO_UNROTATEUP
    }

    override fun update() {
        _servoDifleft.position = _liftTarget / 270.0
        _servoDifRight.position = 1.0 - _liftTarget / 270.0
    }
}
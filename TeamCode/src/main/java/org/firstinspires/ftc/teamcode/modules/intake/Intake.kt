package org.firstinspires.ftc.teamcode.modules.intake

import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.servoAngle.ServoAngle
import org.firstinspires.ftc.teamcode.utils.softServo.SoftServo

object Intake: IRobotModule {
    private lateinit var horizontalServoLeft: SoftServo
    private lateinit var horizontalServoRight: SoftServo
    private lateinit var servoClamp: Servo
    private lateinit var servoFlip: Servo
    private lateinit var servoRotate: Servo
    override fun init(collector: BaseCollector) {
        horizontalServoLeft = SoftServo(collector.devices.horizontalServoLeft, 0.1)
        horizontalServoRight = SoftServo(collector.devices.horizontalServoRight, 0.88)
        servoClamp = collector.devices.servoClamp
        servoFlip = collector.devices.servoFlip
        servoRotate = collector.devices.servoRotate
    }

    var clamp = ClampPosition.SERVO_UNCLAMP
        set(value) {
            if(value == ClampPosition.SERVO_CLAMP){
                servoClamp.position = Configs.IntakeConfig.SERVO_CLAMP
            }
            else if(value == ClampPosition.SERVO_UNCLAMP){
                servoClamp.position = Configs.IntakeConfig.SERVO_UNCLAMP
            }

            field = value
        }

    var flip = GalaxyFlipPosition.SERVO_FLIP
        set(value) {
            if(value == GalaxyFlipPosition.SERVO_FLIP){
                servoFlip.position = Configs.IntakeConfig.SERVO_FLIP
            }
            else if(value == GalaxyFlipPosition.SERVO_UNFLIP){
                servoFlip.position = Configs.IntakeConfig.SERVO_UNFLIP
            }

            field = value;
        }

    var position = AdvancedPosition.SERVO_UNPROMOTED
        set(value) {
            if(value == AdvancedPosition.SERVO_PROMOTED) {
                horizontalServoLeft.targetPosition = Configs.IntakeConfig.SERVO_PROMOTED_LEFT
                horizontalServoRight.targetPosition = Configs.IntakeConfig.SERVO_PROMOTED_RIGHT
            }
            else if(value == AdvancedPosition.SERVO_UNPROMOTED) {
                horizontalServoLeft.targetPosition = Configs.IntakeConfig.SERVO_UNPROMOTED_LEFT
                horizontalServoRight.targetPosition = Configs.IntakeConfig.SERVO_UNPROMOTED_RIGHT
            }

            field = value
        }

    var rotate = rotatePosition.SERVO_UNROTATE
        set(value)
        {
            if(rotate == rotatePosition.SERVO_ROTATE){
                servoRotate.position = Configs.IntakeConfig.SERVO_ROTATE
            }
            else if(rotate == rotatePosition.SERVO_UNROTATE){
                servoRotate.position = Configs.IntakeConfig.SERVO_UNROTATE
            }
            field = value
        }

    enum class AdvancedPosition(double: Double)//нижняя
    {
        SERVO_PROMOTED(20.0),
        SERVO_UNPROMOTED(30.0),
        SERVO_PROMOTED1(20.0),
        SERVO_UNPROMOTED1(30.0)
    }
    enum class ClampPosition(double: Double)// захват
    {
        SERVO_CLAMP(10.0),
        SERVO_UNCLAMP(20.0)
    }
    enum class GalaxyFlipPosition(double: Double)
    {
        SERVO_UNFLIP(10.0),
        SERVO_FLIP(30.0)
    }
    enum class rotatePosition(double: Double)
    {
        SERVO_ROTATE(20.0),
        SERVO_UNROTATE(10.0)
    }
}
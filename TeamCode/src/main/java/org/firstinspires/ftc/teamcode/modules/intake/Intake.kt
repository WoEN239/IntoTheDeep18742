package org.firstinspires.ftc.teamcode.modules.intake

import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.servoAngle.ServoAngle

object Intake: IRobotModule {
    private lateinit var horizontalServoLeft: ServoAngle
    private lateinit var horizontalServoRight: ServoAngle
    private lateinit var servoClamp: ServoAngle
    private lateinit var servoFlip: ServoAngle
    private lateinit var servoRotate: ServoAngle
    override fun init(collector: BaseCollector) {
        horizontalServoLeft = ServoAngle(collector.devices.horizontalServoLeft, 180.0)
        horizontalServoRight = ServoAngle(collector.devices.horizontalServoRight, 180.0)
        servoClamp = ServoAngle(collector.devices.servoClamp, 180.0)
        servoFlip = ServoAngle(collector.devices.servoFlip, 180.0)
        servoRotate = ServoAngle(collector.devices.servoRotate, 180.0)
    }

    var clamp = ClampPosition.SERVO_UNCLAMP
        set(value) {
            if(value == ClampPosition.SERVO_CLAMP){
                servoClamp.angle = Configs.IntakeConfig.SERVO_CLAMP
            }
            else if(value == ClampPosition.SERVO_UNCLAMP){
                servoClamp.angle = Configs.IntakeConfig.SERVO_UNCLAMP
            }

            field = value
        }

    var flip = GalaxyFlipPosition.SERVO_FLIP
        set(value) {
            if(value == GalaxyFlipPosition.SERVO_FLIP){
                servoFlip.angle = Configs.IntakeConfig.SERVO_FLIP
            }
            else if(value == GalaxyFlipPosition.SERVO_UNFLIP){
                servoFlip.angle = Configs.IntakeConfig.SERVO_UNFLIP
            }

            field = value;
        }

    var position = AdvancedPosition.SERVO_UNPROMOTED
        set(value) {
            if(value == AdvancedPosition.SERVO_PROMOTED) {
                horizontalServoLeft.angle = Configs.IntakeConfig.SERVO_PROMOTED
                horizontalServoRight.angle = Configs.IntakeConfig.SERVO_PROMOTED
            }
            else if(value == AdvancedPosition.SERVO_UNPROMOTED) {
                horizontalServoLeft.angle = Configs.IntakeConfig.SERVO_UNPROMOTED
                horizontalServoRight.angle = Configs.IntakeConfig.SERVO_UNPROMOTED
            }

            field = value
        }

    var rotate = rotatePosition.SERVO_UNROTATE
        set(value)
        {
            if(rotate == rotatePosition.SERVO_ROTATE){
                servoRotate.angle = Configs.IntakeConfig.SERVO_ROTATE
            }
            else if(rotate == rotatePosition.SERVO_UNROTATE){
                servoRotate.angle = Configs.IntakeConfig.SERVO_UNROTATE
            }
            field = value
        }

    enum class AdvancedPosition(double: Double)//нижняя
    {
        SERVO_PROMOTED(20.0),
        SERVO_UNPROMOTED(30.0)
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
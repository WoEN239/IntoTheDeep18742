package org.firstinspires.ftc.teamcode.modules.intake

import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.utils.configs.Configs

object Intake: IRobotModule {
    private lateinit var horizontalServoLeft: Servo
    private lateinit var horizontalServoRight: Servo
    private lateinit var servoClamp: Servo
    private lateinit var servoFlip: Servo
    private lateinit var servoRotate: Servo
    override fun init(collector: BaseCollector) {
        horizontalServoLeft = collector.devices.horizontalServoLeft
        horizontalServoRight = collector.devices.horizontalServoRight
        servoClamp = collector.devices.servoClamp
        servoFlip = collector.devices.servoFlip
        servoRotate = collector.devices.servoRotate
    }
    var clamp = ClampPosition.SERVO_UNCLAMP
    var flip = GalaxyFlipPosition.SERVO_FLIP
    var position = AdvancedPosition.SERVO_UNPROMOTED
    var rotate = rotatePosition.SERVO_UNROTATE
        set(value)
        {
            if(clamp == ClampPosition.SERVO_CLAMP){
                servoClamp.position = Configs.IntakeConfig.SERVO_CLAMP
            }
            else if(clamp == ClampPosition.SERVO_UNCLAMP){
                servoClamp.position = Configs.IntakeConfig.SERVO_UNCLAMP
            }
            if(position == AdvancedPosition.SERVO_PROMOTED) {
                horizontalServoLeft.position = Configs.IntakeConfig.SERVO_PROMOTED
                horizontalServoRight.position = Configs.IntakeConfig.SERVO_PROMOTED
            }
           else if(position == AdvancedPosition.SERVO_UNPROMOTED) {
                horizontalServoLeft.position = Configs.IntakeConfig.SERVO_UNPROMOTED
                horizontalServoRight.position = Configs.IntakeConfig.SERVO_UNPROMOTED
            }
            if(flip == GalaxyFlipPosition.SERVO_FLIP){
                servoFlip.position = Configs.IntakeConfig.SERVO_FLIP
            }
            else if(flip == GalaxyFlipPosition.SERVO_UNFLIP){
            servoFlip.position = Configs.IntakeConfig.SERVO_UNFLIP
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
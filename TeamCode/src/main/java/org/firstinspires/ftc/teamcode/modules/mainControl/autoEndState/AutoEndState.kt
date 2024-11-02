package org.firstinspires.ftc.teamcode.modules.mainControl.autoEndState

import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.modules.driveTrain.DriveTrain
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.modules.navigation.gyro.MergeGyro
import org.firstinspires.ftc.teamcode.modules.navigation.odometry.MergeOdometry
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.pidRegulator.PIDRegulator
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.timer.Timer
import org.firstinspires.ftc.teamcode.utils.units.Vec2

object AutoEndState : IRobotModule {
    private lateinit var _lightPopit: DcMotorEx
    private lateinit var _lightPopit1: DcMotorEx
    override fun init(collector: BaseCollector) {
        Intake.clampUp = Intake.ClampPositionUp.SERVO_CLAMPUP
        _lightPopit = collector.devices.lightPopit
        _lightPopit1 = collector.devices.lightPopit1
        _lightPopit.power = 1.0
        _lightPopit1.power = 1.0
    }

    override fun update() {
        _lightPopit.power = 1.0
        _lightPopit1.power = 1.0
        StaticTelemetry.addLine("odometer position = ${MergeOdometry.position}") //odometer positino = Vec2(5.0, 2.0)

        val err = _targetPosition - MergeOdometry.position

        if (!_isStop)
            DriveTrain.drivePowerDirection(
                Vec2(
                    _forwardPid.update(err.x),
                    _sidePid.update(err.y)
                ).turn(-MergeGyro.rotation.angle), 0.0
            )
    }

    private val isAtTarget: Boolean
        get() {
            val err = _targetPosition - MergeOdometry.position

            return err.x < 1.0 && err.y < 1.0
        }

    private val _timer = Timer()

    private var _targetPosition = Vec2(0.0, 0.0)

    private val _forwardPid = PIDRegulator(Configs.DriveTrainConfig.FORWARD_PID)
    private val _sidePid = PIDRegulator(Configs.DriveTrainConfig.SIDE_PID)

    private var _isStop = true

    override fun lateStart(){
        DriveTrain.drivePowerDirection(Vec2(-0.3, 0.0), 0.0)
        _timer.start(5.0){
            DriveTrain.drivePowerDirection(Vec2(0.0, 0.0), 0.0)
        }
        DriveTrain.drivePowerDirection(Vec2(-0.3, 0.0), 0.0)
        _timer.start(5.0){
            DriveTrain.drivePowerDirection(Vec2(0.0, 0.0), 0.0)
        }
    }
}
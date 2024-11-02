package org.firstinspires.ftc.teamcode.modules.mainControl.autoEndState

import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.modules.driveTrain.DriveTrain
import org.firstinspires.ftc.teamcode.modules.navigation.gyro.MergeGyro
import org.firstinspires.ftc.teamcode.modules.navigation.odometry.MergeOdometry
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.pidRegulator.PIDRegulator
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.timer.Timer
import org.firstinspires.ftc.teamcode.utils.units.Vec2

object AutoEndState : IRobotModule {
    override fun init(collector: BaseCollector) {

    }

    override fun update() {
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
    }
}
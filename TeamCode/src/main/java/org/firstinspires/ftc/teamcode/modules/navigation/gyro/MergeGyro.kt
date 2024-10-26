package org.firstinspires.ftc.teamcode.modules.navigation.gyro

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.modules.navigation.odometry.OdometersOdometry
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.exponentialFilter.ExponentialFilter
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.units.Angle

object MergeGyro : IRobotModule {
    private lateinit var imu: IMU

    private val _mergeFilter = ExponentialFilter(Configs.GyroscopeConfig.MERGE_COEF)

    private var _iterations = 0

    override fun init(collector: BaseCollector) {
        imu = collector.devices.imu

        imu.initialize(
            IMU.Parameters(RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP)))
    }

    override fun start() {
        imu.resetYaw()
    }

    var rotation = Angle(0.0)
        get
        private set

    var velocity = 0.0
        get
        private set

    private var _oldRotateGyro = Angle(0.0)

    override fun update() {
        _mergeFilter.coef = Configs.GyroscopeConfig.MERGE_COEF

        val odometerTurn = OdometerGyro.calculateRotate()
        val gyroTurn = IMUGyro.calculateRotate().angle

        rotation = Angle(_mergeFilter.updateRaw(gyroTurn, odometerTurn.angle - gyroTurn))

        velocity = OdometerGyro.calculateRotateVelocity()

        StaticTelemetry.addData("robot merge rotate", rotation.toDegree())
        StaticTelemetry.addData("robot odometer rotate", odometerTurn.toDegree())
        StaticTelemetry.addData("robot gyro rotate",  Math.toDegrees(gyroTurn))
    }
}
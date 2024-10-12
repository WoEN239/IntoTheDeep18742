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

object Gyro : IRobotModule {
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

    private var _oldRotateOdometry = Angle(0.0)

    override fun update() {
        _mergeFilter.coef = Configs.GyroscopeConfig.MERGE_COEF

        if (Configs.GyroscopeConfig.USE_ODOMETRY) {
            val odometerTurn = OdometersOdometry.calculateRotate()

            if (_iterations > Configs.GyroscopeConfig.GYRO_ITERATIONS) {
                rotation = Angle(_mergeFilter.updateRaw(OdometersOdometry.calculateRotate().angle, odometerTurn.angle - imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)))

                _iterations = 0
            } else
                rotation += odometerTurn - _oldRotateOdometry

            _iterations++

            _oldRotateOdometry = odometerTurn

            velocity = OdometersOdometry.calculateRotateVelocity()
        } else {
            rotation = Angle(imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS))
            velocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate.toDouble()
        }

        StaticTelemetry.addData("robot rotate", rotation.toDegree())
    }
}
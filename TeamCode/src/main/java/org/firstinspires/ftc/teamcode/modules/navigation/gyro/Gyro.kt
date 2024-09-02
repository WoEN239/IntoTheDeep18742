package org.firstinspires.ftc.teamcode.modules.navigation.gyro

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.units.Angle


object Gyro : IRobotModule {
    private lateinit var imu: IMU

    override fun init(collector: BaseCollector) {
        imu = collector.devices.imu

        imu.initialize(
            IMU.Parameters(
                RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
            )
        )
    }

    override fun start() {
        imu.resetYaw()
    }

    var rotation = Angle(0.0)
        get private set

    var velocity = 0.0
        get private set

    override fun update() {
        rotation.angle = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)

        velocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate.toDouble()

        StaticTelemetry.addData("gyro rotate", rotation.toDegree())
    }
}
package org.firstinspires.ftc.teamcode.modules.navigation.gyro

import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.modules.navigation.odometry.HardwareOdometers
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.units.Angle

object OdometerGyro: IRobotModule {
    override fun init(collector: BaseCollector) {}

    fun calculateRotate() =
        Angle((HardwareOdometers.forwardOdometerLeftPosition / Configs.OdometryConfig.FORWARD_ODOMETER_LEFT_RADIUS - HardwareOdometers.forwardOdometerRightPosition / Configs.OdometryConfig.FORWARD_ODOMETER_RIGHT_RADIUS) / 2.0)

    fun calculateRotateVelocity() =
        (HardwareOdometers.forwardOdometerLeftVelocity / Configs.OdometryConfig.FORWARD_ODOMETER_LEFT_RADIUS + HardwareOdometers.forwardOdometerRightVelocity / Configs.OdometryConfig.FORWARD_ODOMETER_RIGHT_RADIUS) / 2.0
}
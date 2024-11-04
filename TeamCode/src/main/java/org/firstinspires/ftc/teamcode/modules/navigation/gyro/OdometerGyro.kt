package org.firstinspires.ftc.teamcode.modules.navigation.gyro

import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.modules.navigation.odometry.HardwareOdometers
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.units.Angle

object OdometerGyro: IRobotModule {
    override fun init(collector: BaseCollector, bus: EventBus) {}

    fun calculateRotate() =
        Angle((HardwareOdometers.forwardOdometerRightPosition / Configs.OdometryConfig.FORWARD_ODOMETER_RIGHT_RADIUS - HardwareOdometers.forwardOdometerLeftPosition / Configs.OdometryConfig.FORWARD_ODOMETER_LEFT_RADIUS) / 2.0)

    fun calculateRotateVelocity() =
        (HardwareOdometers.forwardOdometerRightVelocity / Configs.OdometryConfig.FORWARD_ODOMETER_RIGHT_RADIUS - HardwareOdometers.forwardOdometerLeftVelocity / Configs.OdometryConfig.FORWARD_ODOMETER_LEFT_RADIUS) / 2.0
}
package org.firstinspires.ftc.teamcode.modules.navigation.gyro

import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.modules.navigation.HardwareOdometers
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.units.Angle

class OdometerGyro : IRobotModule {
    override fun init(collector: BaseCollector, bus: EventBus) {
        bus.subscribe(HardwareOdometers.UpdateHardwareOdometersEvent::class) {
            bus.invoke(
                UpdateOdometerGyroEvent(
                    Angle(
                        (it.rightPosition / Configs.OdometryConfig.FORWARD_ODOMETER_RIGHT_RADIUS - it.leftPosition / Configs.OdometryConfig.FORWARD_ODOMETER_LEFT_RADIUS) / 2.0) + collector.parameters.oldStartPosition.angle,
                    (it.rightVelocity / Configs.OdometryConfig.FORWARD_ODOMETER_RIGHT_RADIUS - it.leftVelocity / Configs.OdometryConfig.FORWARD_ODOMETER_LEFT_RADIUS) / 2
                )
            )
        }
    }

    class UpdateOdometerGyroEvent(val rotate: Angle, val velocity: Double) : IEvent
}
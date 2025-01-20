package org.firstinspires.ftc.teamcode.modules.navigation.odometry

import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.modules.navigation.HardwareOdometers
import org.firstinspires.ftc.teamcode.modules.navigation.gyro.MergeGyro
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Color
import org.firstinspires.ftc.teamcode.utils.units.Vec2

class OdometersOdometry : IRobotModule {
    override fun init(collector: BaseCollector, bus: EventBus) {
        _position = collector.gameSettings.startPosition.position

        bus.subscribe(HardwareOdometers.UpdateHardwareOdometersEvent::class) {
            val gyro = bus.invoke(MergeGyro.RequestMergeGyroEvent())

            val deltaLeftPosition = it.leftPosition - it.leftPositionOld
            val deltaRightPosition = it.rightPosition - it.rightPositionOld
            val deltaSidePosition = it.sidePosition - it.sidePositionOld
            val deltaRotate = gyro.odometerRotate!! - _oldOdometerRotate

            _oldOdometerRotate = gyro.odometerRotate!!

            _position += Vec2(
                (deltaRightPosition + deltaLeftPosition) / 2.0,
                deltaSidePosition - (Configs.OdometryConfig.SIDE_ODOMETER_RADIUS * deltaRotate.angle)
            ).turn(gyro.rotation!!.angle - (gyro.oldRotation!!.angle - gyro.rotation!!.angle) * 0.5)

            val velocity = Vec2(
                (it.leftVelocity + it.rightVelocity) / 2.0,
                it.sideVelocity - Configs.OdometryConfig.SIDE_ODOMETER_RADIUS * gyro.velocity!!
            )

            bus.invoke(UpdateOdometersOdometryEvent(_position, velocity))
        }
    }

    private var _position = Vec2.ZERO
    private var _oldOdometerRotate = Angle.ZERO

    class UpdateOdometersOdometryEvent(val position: Vec2, val velocity: Vec2): IEvent
}
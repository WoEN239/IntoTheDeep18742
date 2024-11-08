package org.firstinspires.ftc.teamcode.modules.navigation.odometry

import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.modules.navigation.gyro.MergeGyro
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Vec2

class OdometersOdometry : IRobotModule {
    private var _rotation = Angle(0.0)
    private var _oldRotation = Angle(0.0)
    private var _rotateVelocity = 0.0

    override fun init(collector: BaseCollector, bus: EventBus) {
        bus.subscribe(HardwareOdometers.UpdateHardwareOdometersEvent::class) {
            val deltaLeftPosition = it.leftPosition - it.leftPositionOld
            val deltaRightPosition = it.rightPosition - it.rightPositionOld
            val deltaSidePosition = it.sidePosition - it.sidePositionOld
            val deltaRotate = _rotation - _oldRotation

            _position += Vec2(
                (deltaRightPosition + deltaLeftPosition) / 2.0,
                deltaSidePosition - (Angle(Configs.OdometryConfig.SIDE_ODOMETER_RADIUS) * deltaRotate).angle
            ).turn(_rotation.angle)

            val velocity = Vec2(
                (it.leftVelocity + it.rightVelocity) / 2.0,
                it.sideVelocity - Configs.OdometryConfig.SIDE_ODOMETER_RADIUS * _rotateVelocity
            )
            bus.invoke(UpdateOdometersOdometryEvent(_position, velocity))
        }

        bus.subscribe(MergeGyro.UpdateMergeGyroEvent::class){
            _rotation = it.rotation
            _oldRotation = it.oldRotation
            _rotateVelocity = it.velocity
        }
    }

    private var _position = Vec2.ZERO

    class UpdateOdometersOdometryEvent(val position: Vec2, val velocity: Vec2): IEvent
}
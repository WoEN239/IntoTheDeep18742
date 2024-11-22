package org.firstinspires.ftc.teamcode.modules.navigation.odometry

import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.modules.navigation.gyro.MergeGyro
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Color
import org.firstinspires.ftc.teamcode.utils.units.Vec2

class MergeOdometry: IRobotModule {
    private var _rotation = Angle(0.0)

    override fun init(collector: BaseCollector, bus: EventBus) {
        bus.subscribe(OdometersOdometry.UpdateOdometersOdometryEvent::class){
            _position = it.position
            _velocity = it.velocity

            bus.invoke(UpdateMergeOdometryEvent(_position, _velocity))

            StaticTelemetry.drawRect(it.position, Vec2(30.0, 30.0), _rotation.angle, Color.BLUE)
            StaticTelemetry.addData("odometerPosition", it.position)
            StaticTelemetry.addData("odomeeterVelocity", it.velocity)
        }

        bus.subscribe(MergeGyro.UpdateMergeGyroEvent::class){
            _rotation = it.rotation
        }

        bus.subscribe(RequestMergePositionEvent::class){
            it.position = _position
            it.velocity = _velocity
        }
    }

    private var _position = Vec2.ZERO
    private var _velocity = Vec2.ZERO

    class UpdateMergeOdometryEvent(val position: Vec2, val velocity: Vec2): IEvent
    class RequestMergePositionEvent(var position: Vec2? = null, var velocity: Vec2? = null): IEvent
}
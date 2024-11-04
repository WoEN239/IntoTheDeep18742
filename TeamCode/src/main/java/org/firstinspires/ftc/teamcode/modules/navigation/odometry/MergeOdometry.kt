package org.firstinspires.ftc.teamcode.modules.navigation.odometry

import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.units.Color
import org.firstinspires.ftc.teamcode.utils.units.Vec2

class MergeOdometry: IRobotModule {
    override fun init(collector: BaseCollector, bus: EventBus) {
        bus.subscribe(OdometersOdometry.UpdateOdometersOdometryEvent::class){
            bus.invoke(UpdateMergeOdometryEvent(it.position, it.velocity))
        }
    }

    var position = Vec2.ZERO
    var velocity = Vec2.ZERO

    class UpdateMergeOdometryEvent(val position: Vec2, val velocity: Vec2): IEvent
}
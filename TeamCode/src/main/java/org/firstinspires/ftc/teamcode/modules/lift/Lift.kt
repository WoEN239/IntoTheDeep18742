package org.firstinspires.ftc.teamcode.modules.lift

import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.modules.navigation.gyro.OdometerGyro
import org.firstinspires.ftc.teamcode.modules.navigation.gyro.OdometerGyro.UpdateOdometerGyroEvent

class Lift: IRobotModule {
    override fun init(collector: BaseCollector, bus: EventBus) {
        bus.subscribe(ABOBAEvent::class){

        }

        bus.subscribe(UpdateOdometerGyroEvent::class){

        }

        bus.invoke(ABOBAEvent(5.0))
    }

    class ABOBAEvent(val a: Double): IEvent

    override fun update() {
        super.update()
    }
}
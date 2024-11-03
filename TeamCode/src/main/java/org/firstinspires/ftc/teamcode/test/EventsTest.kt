package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry

@Disabled
@TeleOp
class EventsTest: LinearOpMode() {
    class A(val str: String): IEvent

    override fun runOpMode() {
        StaticTelemetry.setPhoneTelemetry(telemetry)

        val bus = EventBus()

        bus.subscribe(A::class) { StaticTelemetry.addLine(it.str + " 1") }
        bus.subscribe(A::class) { StaticTelemetry.addLine(it.str + " 2") }
        bus.subscribe(A::class) { StaticTelemetry.addLine(it.str + " 3") }

        waitForStart()
        resetRuntime()

        bus.invoke(A("work"))

        while (opModeIsActive()){
            StaticTelemetry.update()
        }
    }
}
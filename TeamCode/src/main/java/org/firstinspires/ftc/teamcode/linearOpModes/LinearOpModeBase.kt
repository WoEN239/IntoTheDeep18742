package org.firstinspires.ftc.teamcode.linearOpModes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry

open class LinearOpModeBase: LinearOpMode() {
    override fun runOpMode() {
        StaticTelemetry.setPhoneTelemetry(telemetry)

        try {
            waitForStart()
            resetRuntime()

            while (opModeIsActive()) {
                StaticTelemetry.update()
            }
        }
        catch (e: Exception){
            StaticTelemetry.addLine(e.message!!)

            for (i in e.stackTrace)
                StaticTelemetry.addLine(i.javaClass.`package`!!.name)
        }
    }
}
package org.firstinspires.ftc.teamcode.linearOpModes

import android.os.Environment
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry

open class LinearOpModeBase: LinearOpMode() {
    protected open fun getCollector() = BaseCollector(this)

    override fun runOpMode() {
        StaticTelemetry.setPhoneTelemetry(telemetry)

        try {
            val collector = getCollector()

            collector.init()

            waitForStart()
            resetRuntime()

            collector.start()

            while (opModeIsActive()) {
                StaticTelemetry.update()

                collector.update()
            }

            collector.stop()
        }
        catch (e: Exception){
            StaticTelemetry.addLine(e.message!!)

            for (i in e.stackTrace)
                StaticTelemetry.addLine(i.fileName )

            StaticTelemetry.update()

            throw e
        }
    }
}
package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.utils.devices.Battery
import org.firstinspires.ftc.teamcode.utils.softServo.SoftServo
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.timer.Timer
import org.firstinspires.ftc.teamcode.utils.updateListener.UpdateHandler


@TeleOp
class Test: LinearOpMode() {
    override fun runOpMode() {
        StaticTelemetry.setPhoneTelemetry(telemetry)

        try {
            val handler = UpdateHandler()

            val battery = Battery(hardwareMap.get(VoltageSensor::class.java, "Control Hub"))

            val softServo = SoftServo(hardwareMap.get("servo") as Servo)

            val timer = Timer()

            handler.init(battery)

            waitForStart()
            resetRuntime()

            handler.start()

            var currentPos = 0.0

            var a = {}

            a = {
                currentPos = 0.0

                timer.start({!softServo.isEnd}, {
                    currentPos = 1.0

                    timer.start({!softServo.isEnd}, a)
                })
            }

            timer.start({!softServo.isEnd}, a)

            while (opModeIsActive()) {
                battery.update()
                StaticTelemetry.update()
                handler.update()

                softServo.targetPosition = currentPos
            }

            handler.stop()
        }
        catch (e: Exception){
            StaticTelemetry.addLine(e.message!!)

            for (i in e.stackTrace)
                StaticTelemetry.addLine(i.javaClass.`package`!!.name)

            StaticTelemetry.update()

            throw e
        }
    }

}
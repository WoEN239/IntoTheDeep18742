package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.utils.contServo.ContServo
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

            val serv = SoftServo(hardwareMap.get("servoClamp") as ServoImplEx, 0.88)

            val timer = Timer()

            handler.init(BaseCollector.InitContext(battery))

            waitForStart()
            resetRuntime()

            handler.start()

            var currentPos = 0.26

            var a = {}

            a = {
                currentPos = 0.88

                timer.start({!serv.isEnd}, {
                    currentPos = 0.26

                    timer.start({!serv.isEnd}, a)
                })
            }

            timer.start({!serv.isEnd}, a)

            while (opModeIsActive()) {
                battery.update()
                StaticTelemetry.update()
                handler.update()

                serv.targetPosition = currentPos
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
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

            val servRight = SoftServo(hardwareMap.get("horizontalServoRight") as ServoImplEx, 0.88)
            val servLeft = SoftServo(hardwareMap.get("horizontalServoLeft") as ServoImplEx, 0.1)

            val timer = Timer()

            handler.init(BaseCollector.InitContext(battery))

            waitForStart()
            resetRuntime()

            handler.start()

            var currentRightPos = 0.26
            var currentLeftPos = 0.72

            var a = {}

            // 0.88 0.26
            // 0.1 0.72

            a = {
                currentRightPos = 0.88
                currentLeftPos = 0.1

                timer.start({!servRight.isEnd || !servLeft.isEnd}, {
                    currentRightPos = 0.26
                    currentLeftPos = 0.72

                    timer.start({!servRight.isEnd || !servLeft.isEnd}, a)
                })
            }

            timer.start({!servRight.isEnd || !servLeft.isEnd}, a)

            while (opModeIsActive()) {
                battery.update()
                StaticTelemetry.update()
                handler.update()

                servRight.targetPosition = currentRightPos
                servLeft.targetPosition = currentLeftPos
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
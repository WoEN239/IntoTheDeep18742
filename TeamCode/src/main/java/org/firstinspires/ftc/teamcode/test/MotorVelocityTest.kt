package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.utils.motor.Motor
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.timer.Timer
import org.firstinspires.ftc.teamcode.utils.updateListener.UpdateHandler


@TeleOp
class MotorVelocityTest: LinearOpMode() {
    override fun runOpMode() {
        StaticTelemetry.setPhoneTelemetry(telemetry)

        try {
            val handler = UpdateHandler()

            val motor = hardwareMap.get("motor") as DcMotorEx
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

            val motorPidf = Motor(motor)
            val timer = Timer()

            waitForStart()
            resetRuntime()

            handler.start()

            var a = {}

            motorPidf.targetPower = 0.5

            a = {
                motorPidf.targetPower = 0.9
                timer.start(1.2) {
                    motorPidf.targetPower = 0.2

                    timer.start(1.2) {
                        motorPidf.targetPower = 0.5

                        timer.start(1.2, a)
                    }
                }
            }

            timer.start(
                1.2,
                a
            )

            while (opModeIsActive()) {
                StaticTelemetry.update()
                handler.update()
            }

            handler.stop()
        }
        catch (e: Exception){
            StaticTelemetry.addLine(e.message!!)

            for (i in e.stackTrace)
                StaticTelemetry.addLine(i.javaClass.`package`!!.name)

            throw e
        }
    }

}
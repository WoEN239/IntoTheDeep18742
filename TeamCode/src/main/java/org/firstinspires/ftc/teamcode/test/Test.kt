package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.utils.devices.Battery
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.updateListener.UpdateHandler


@TeleOp
class Test: LinearOpMode() {
    override fun runOpMode() {
        StaticTelemetry.setPhoneTelemetry(telemetry)

        try {
            val handler = UpdateHandler()

            val battery = Battery(hardwareMap.get(VoltageSensor::class.java, "Control Hub"))

            handler.init(BaseCollector.InitContext(battery))

            val rightLiftMotor = hardwareMap.get("liftMotorRight") as DcMotorEx
            val leftLiftMotor = hardwareMap.get("liftMotorLeft") as DcMotorEx

            waitForStart()
            resetRuntime()

            handler.start()

            while (opModeIsActive()) {
                battery.update()
                StaticTelemetry.update()
                handler.update()

                StaticTelemetry.addLine("leftEnc = " + leftLiftMotor.currentPosition)
                StaticTelemetry.addLine("rightEnc = " + rightLiftMotor.currentPosition)
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
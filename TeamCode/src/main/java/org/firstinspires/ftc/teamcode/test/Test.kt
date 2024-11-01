package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
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

            val rightMotor = hardwareMap.get("liftMotorRight") as DcMotor
            val leftMotor = hardwareMap.get("liftMotorLeft") as DcMotor

            rightMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

            leftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

            rightMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            leftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

            rightMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            leftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

            leftMotor.direction = DcMotorSimple.Direction.REVERSE

            waitForStart()
            resetRuntime()

            handler.start()

            //верхняя левая 2940 правая 2940
            //средняя

            while (opModeIsActive()) {
                battery.update()
                StaticTelemetry.update()
                handler.update()

                StaticTelemetry.addLine("leftPos = " + leftMotor.currentPosition)
                StaticTelemetry.addLine("rightPos = " + rightMotor.currentPosition)
            }

            handler.stop()
        }
        catch (e: Exception){
            StaticTelemetry.addLine(e.message!!)

            for (i in e.stackTrace)
                StaticTelemetry.addLine(i.fileName)

            StaticTelemetry.update()

            throw e
        }
    }

}
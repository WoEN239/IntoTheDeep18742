package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.contServo.ContServo
import org.firstinspires.ftc.teamcode.utils.devices.Battery
import org.firstinspires.ftc.teamcode.utils.softServo.SoftServo
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.timer.Timer
import org.firstinspires.ftc.teamcode.utils.updateListener.UpdateHandler


@TeleOp
class Test: LinearOpMode() {
    private lateinit var _servoDifleft: Servo
    private lateinit var _servoDifRight: Servo

    @Config
    internal object TestConfigs{
        @JvmField
        var X_ROT = 0.0

        @JvmField
        var Y_ROT = 0.0
    }

    fun setDifPos(xRot: Double, yRot: Double)
    {
        val x = xRot + 135
        val y = yRot + 10

        _servoDifRight.position = clamp((y + x) / Configs.IntakeConfig.MAX, 0.0, 1.0)
        _servoDifleft.position = clamp(1.0 - (x - y) / Configs.IntakeConfig.MAX, 0.0, 1.0)
    }

    override fun runOpMode() {
        StaticTelemetry.setPhoneTelemetry(telemetry)

        try {
            _servoDifleft = this.hardwareMap.get("servoDifLeft") as Servo
            _servoDifRight = this.hardwareMap.get("servoDifRight") as Servo

            val handler = UpdateHandler()

            val battery = Battery(hardwareMap.get(VoltageSensor::class.java, "Control Hub"))

            handler.init(BaseCollector.InitContext(battery))

            waitForStart()
            resetRuntime()

            handler.start()


            while (opModeIsActive()) {
                battery.update()
                StaticTelemetry.update()
                handler.update()

                setDifPos(TestConfigs.X_ROT, TestConfigs.Y_ROT)
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
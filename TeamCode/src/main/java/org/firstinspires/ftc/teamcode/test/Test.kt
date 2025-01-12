package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.modules.camera.StickProcessor
import org.firstinspires.ftc.teamcode.modules.intake.Lift
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.contServo.ContServo
import org.firstinspires.ftc.teamcode.utils.devices.Battery
import org.firstinspires.ftc.teamcode.utils.softServo.SoftServo
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.timer.Timer
import org.firstinspires.ftc.teamcode.utils.timer.Timers
import org.firstinspires.ftc.teamcode.utils.updateListener.UpdateHandler
import org.firstinspires.ftc.vision.VisionPortal
import kotlin.math.PI


@TeleOp
class Test: LinearOpMode() {
    @Config
    internal object TestConfigs{
        @JvmField
        var X_POS = 0.0

        @JvmField
        var Y_POS = 0.0
    }

    override fun runOpMode() {
        StaticTelemetry.setPhoneTelemetry(telemetry)

        try {
            val handler = UpdateHandler()
            val timers = Timers()

            val battery = Battery(hardwareMap.get(VoltageSensor::class.java, "Control Hub"))
            val _servoDifRight = hardwareMap.get("servoDifRight") as ServoImplEx
            val _servoDifLeft = hardwareMap.get("servoDifLeft") as ServoImplEx

            _servoDifLeft.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
            _servoDifRight.pwmRange = PwmControl.PwmRange(500.0, 2500.0)

            fun setDifPos(xRot: Double, yRot: Double)
            {
                val x = xRot + 135.0
                val y = yRot * Configs.IntakeConfig.GEAR_RATIO

                _servoDifRight.position = clamp((y + x) / Configs.IntakeConfig.MAX, 0.0, 1.0)
                _servoDifLeft.position = clamp(1.0 - (x - y) / Configs.IntakeConfig.MAX, 0.0, 1.0)
            }

            handler.init(BaseCollector.InitContext(battery))

            waitForStart()
            resetRuntime()

            handler.start()

            while (opModeIsActive()) {
                battery.update()
                StaticTelemetry.update()
                handler.update()
                timers.update()

                setDifPos(TestConfigs.X_POS, TestConfigs.Y_POS)
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
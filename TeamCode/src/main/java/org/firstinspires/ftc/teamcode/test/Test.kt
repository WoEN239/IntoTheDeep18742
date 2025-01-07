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

    }

    override fun runOpMode() {
        StaticTelemetry.setPhoneTelemetry(telemetry)

        try {
            val testServo = hardwareMap.get("testServo") as Servo

            val handler = UpdateHandler()
            val timers = Timers()

            val battery = Battery(hardwareMap.get(VoltageSensor::class.java, "Control Hub"))

            handler.init(BaseCollector.InitContext(battery))

            val processor = StickProcessor()

            val visionPortal = VisionPortal.Builder().addProcessors(processor).setCamera(hardwareMap.get("Webcam 1") as WebcamName).build()

            FtcDashboard.getInstance().startCameraStream(processor, 60.0)

            waitForStart()
            resetRuntime()

            handler.start()

            while (opModeIsActive()) {
                battery.update()
                StaticTelemetry.update()
                handler.update()
                timers.update()

                StaticTelemetry.addData("camera fps", visionPortal.fps)

                val sticks = processor.allianceSticks.get()

                if (sticks.size > 0) {
                    var targetStick = sticks[0]
                    
                    StaticTelemetry.addData("targetStickPos", targetStick.pos)

                    testServo.position = targetStick.angl.angle / PI
                }
                else
                    testServo.position = 0.0
            }

            handler.stop()

            visionPortal.stopStreaming()
            FtcDashboard.getInstance().stopCameraStream()
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
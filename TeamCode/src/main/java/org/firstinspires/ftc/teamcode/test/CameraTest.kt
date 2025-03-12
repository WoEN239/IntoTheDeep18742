package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.modules.camera.NewStickProcessor
import org.firstinspires.ftc.vision.VisionPortal

@TeleOp
class CameraTest: LinearOpMode() {
    override fun runOpMode() {
        val processor = NewStickProcessor()

        val visionPortalBuilder =
            VisionPortal.Builder().addProcessor(processor).setCamera(hardwareMap.get("Webcam 1") as WebcamName).build()

        FtcDashboard.getInstance().startCameraStream(processor, 30.0)

        waitForStart()
        resetRuntime()

        while (opModeIsActive()){
            FtcDashboard.getInstance().telemetry.update()
        }

        FtcDashboard.getInstance().stopCameraStream()
    }
}
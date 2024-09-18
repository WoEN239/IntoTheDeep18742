package org.firstinspires.ftc.teamcode.modules.camera

import com.acmerobotics.dashboard.FtcDashboard
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.vision.VisionPortal


object Camera : IRobotModule {
    private lateinit var _processor: StickProcessor
    private lateinit var _visionPortal: VisionPortal

    override fun init(collector: BaseCollector) {
        _processor = StickProcessor()

        _visionPortal = VisionPortal.Builder().addProcessors(_processor).setCamera(collector.devices.camera).build()

        FtcDashboard.getInstance().startCameraStream(_processor, 60.0)
    }

    override fun stop() {
        _visionPortal.stopStreaming()
        FtcDashboard.getInstance().stopCameraStream()
    }
}
package org.firstinspires.ftc.teamcode.modules.camera

import com.acmerobotics.dashboard.FtcDashboard
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.utils.units.Orientation
import org.firstinspires.ftc.vision.VisionPortal


class Camera : IRobotModule {
    class RequestRedDetectedSticks(var sticks: Array<Orientation>? = null): IEvent
    class RequestBlueDetectedSticks(var sticks: Array<Orientation>? = null): IEvent
    class SetStickDetectEnable(val enable: Boolean): IEvent

    private lateinit var _processor: StickProcessor
    private lateinit var _visionPortal: VisionPortal

    override fun init(collector: BaseCollector, bus: EventBus) {
        bus.subscribe(RequestRedDetectedSticks::class){
            it.sticks = _processor.redSticks.get()
        }

        bus.subscribe(RequestBlueDetectedSticks::class){
            it.sticks = _processor.blueSticks.get()
        }

        bus.subscribe(SetStickDetectEnable::class){
            _processor.enableDetect.set(it.enable)
        }

        _processor = StickProcessor()

        _visionPortal = VisionPortal.Builder().addProcessors(_processor).setCamera(collector.devices.camera).build()

        FtcDashboard.getInstance().startCameraStream(_processor, 30.0)
    }

    override fun stop() {
        _visionPortal.stopStreaming()
        FtcDashboard.getInstance().stopCameraStream()
    }
}
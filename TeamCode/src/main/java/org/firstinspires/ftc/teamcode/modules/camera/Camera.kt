package org.firstinspires.ftc.teamcode.modules.camera

import com.acmerobotics.dashboard.FtcDashboard
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.utils.units.Orientation
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.VisionProcessor


class Camera : IRobotModule {
    class RequestAllianceDetectedSticks(var sticks: Array<Orientation>? = null): IEvent
    class RequestYellowDetectedSticks(var sticks: Array<Orientation>? = null): IEvent
    class SetStickDetectEnable(val enable: Boolean): IEvent
    class AddCameraProcessor(val processor: VisionProcessor): IEvent

    private lateinit var _processor: StickProcessor
    private lateinit var _visionPortal: VisionPortal

    private var _visionPortalBuilder = VisionPortal.Builder()

    override fun init(collector: BaseCollector, bus: EventBus) {
        bus.subscribe(RequestAllianceDetectedSticks::class){
            it.sticks = _processor.allianceSticks.get()
        }

        bus.subscribe(RequestYellowDetectedSticks::class){
            it.sticks = _processor.yellowSticks.get()
        }

        bus.subscribe(SetStickDetectEnable::class){
            _processor.enableDetect.set(it.enable)
        }

        bus.subscribe(AddCameraProcessor::class){
            _visionPortalBuilder.addProcessor(it.processor)
        }

        _processor = StickProcessor()

        _processor.gameColor.set(collector.parameters.oldStartPosition.color)

        //_visionPortalBuilder = _visionPortalBuilder.addProcessor(_processor).setCamera(collector.devices.camera)
    }

    override fun start() {
        _visionPortal = _visionPortalBuilder.build()
        FtcDashboard.getInstance().startCameraStream(_processor, 30.0)
        _processor.enableDetect.set(true)
    }

    override fun stop() {
        _visionPortal.stopStreaming()
        FtcDashboard.getInstance().stopCameraStream()
    }
}
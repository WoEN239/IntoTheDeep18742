package org.firstinspires.ftc.teamcode.modules.navigation.gyro

import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.exponentialFilter.ExponentialFilter
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.units.Angle

class MergeGyro : IRobotModule {
    private val _mergeFilter = ExponentialFilter(Configs.GyroscopeConfig.MERGE_COEF)

    private var _oldOdometerRotate = Angle.ZERO

    private var _oldMergeRotation = Angle.ZERO

    private var _mergeRotate = Angle.ZERO
    private var _velocity = 0.0

    override fun init(collector: BaseCollector, bus: EventBus) {
        _mergeRotate = collector.gameSettings.startPosition.angle
        _oldMergeRotation = collector.gameSettings.startPosition.angle

        bus.subscribe(IMUGyro.UpdateImuGyroEvent::class){
            _mergeRotate = Angle(_mergeFilter.updateRaw(_mergeRotate.angle, (it.rotate - _mergeRotate).angle))
        }

        bus.subscribe(OdometerGyro.UpdateOdometerGyroEvent::class){
            _oldMergeRotation = _mergeRotate
            _mergeRotate += it.rotate - _oldOdometerRotate

            _oldOdometerRotate = it.rotate

            _velocity = it.velocity

            StaticTelemetry.addData("robot merge rotate", _mergeRotate.toDegree())
        }

        bus.subscribe(RequestMergeGyroEvent::class){
            it.rotation = _mergeRotate
            it.velocity = _velocity
            it.oldRotation = _oldMergeRotation
        }
    }

    class RequestMergeGyroEvent(var rotation: Angle? = null, var oldRotation: Angle? = null, var velocity: Double? = null): IEvent

    override fun update() {
        _mergeFilter.coef = Configs.GyroscopeConfig.MERGE_COEF
    }
}
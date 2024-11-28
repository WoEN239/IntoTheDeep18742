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

    private var _oldRotation = Angle.ZERO

    private var _rotation = Angle.ZERO
    private var _velocity = 0.0

    override fun init(collector: BaseCollector, bus: EventBus) {
        bus.subscribe(IMUGyro.UpdateImuGyroEvent::class){
            _rotation = Angle(_mergeFilter.updateRaw(_rotation.angle, (it.rotate - _rotation).angle))
        }

        bus.subscribe(OdometerGyro.UpdateOdometerGyroEvent::class){
            _rotation += it.rotate - _oldOdometerRotate

            _oldOdometerRotate = it.rotate

            bus.invoke(UpdateMergeGyroEvent(_rotation, _oldRotation, it.velocity))

            _velocity = it.velocity
            _oldRotation = _rotation

            StaticTelemetry.addData("robot merge rotate", _rotation.toDegree())
        }

        bus.subscribe(RequestMergeRotateEvent::class){
            it.rotation = _rotation
            it.velocity = _velocity
        }
    }

    class UpdateMergeGyroEvent(val rotation: Angle, val oldRotation: Angle, val velocity: Double): IEvent
    class RequestMergeRotateEvent(var rotation: Angle? = null, var velocity: Double? = null): IEvent

    override fun update() {
        _mergeFilter.coef = Configs.GyroscopeConfig.MERGE_COEF
    }
}
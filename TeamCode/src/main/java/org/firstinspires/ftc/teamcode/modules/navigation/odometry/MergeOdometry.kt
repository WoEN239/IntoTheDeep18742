package org.firstinspires.ftc.teamcode.modules.navigation.odometry

import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.modules.navigation.gyro.MergeGyro
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.exponentialFilter.ExponentialFilter
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Color
import org.firstinspires.ftc.teamcode.utils.units.Vec2

class MergeOdometry: IRobotModule {
    private var _oldOdometrPos = Vec2.ZERO

    private val _mergeFilterX = ExponentialFilter(Configs.CVOdometryConfig.MERGE_COEF)
    private val _mergeFilterY = ExponentialFilter(Configs.CVOdometryConfig.MERGE_COEF)

    private lateinit var _eventBus: EventBus

    override fun init(collector: BaseCollector, bus: EventBus) {
        _eventBus = bus
        _oldOdometrPos = collector.gameSettings.startPosition.position

        bus.subscribe(CVOdometry.UpdateCVOdometryEvent::class){
            _position.x = _mergeFilterX.updateRaw(_position.x, it.pos.x - _position.x)
            _position.y = _mergeFilterY.updateRaw(_position.y, it.pos.y - _position.y)
        }

        bus.subscribe(OdometersOdometry.UpdateOdometersOdometryEvent::class){
            val deltaPos = it.position - _oldOdometrPos
            _oldOdometrPos = it.position

            _position += deltaPos

            bus.invoke(UpdateMergeOdometryEvent(_position, _velocity))
            StaticTelemetry.addData("odometerPosition", it.position)
        }

        bus.subscribe(RequestMergePositionEvent::class){
            it.position = _position
            it.velocity = _velocity
        }
    }

    override fun update() {
        _mergeFilterX.coef = Configs.CVOdometryConfig.MERGE_COEF
        _mergeFilterY.coef = Configs.CVOdometryConfig.MERGE_COEF

        StaticTelemetry.drawRect(_position, Configs.TelemetryConfig.ROBOT_SIZE, _eventBus.invoke(MergeGyro.RequestMergeGyroEvent()).rotation!!.angle, Color.BLUE)
    }

    private var _position = Vec2.ZERO
    private var _velocity = Vec2.ZERO

    class UpdateMergeOdometryEvent(val position: Vec2, val velocity: Vec2): IEvent
    class RequestMergePositionEvent(var position: Vec2? = null, var velocity: Vec2? = null): IEvent
}
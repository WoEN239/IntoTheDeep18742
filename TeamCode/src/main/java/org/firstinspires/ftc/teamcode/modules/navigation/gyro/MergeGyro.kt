package org.firstinspires.ftc.teamcode.modules.navigation.gyro

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.modules.navigation.odometry.OdometersOdometry
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.exponentialFilter.ExponentialFilter
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.units.Angle

class MergeGyro : IRobotModule {
    private val _mergeFilter = ExponentialFilter(Configs.GyroscopeConfig.MERGE_COEF)

    private var _gyroRotate = Angle(0.0)

    private var _oldRotation = Angle(0.0)

    private var _rotation = Angle.ZERO
    private var _velocity = 0.0

    override fun init(collector: BaseCollector, bus: EventBus) {
        bus.subscribe(IMUGyro.UpdateImuGyroEvent::class){
            _gyroRotate = it.rotate
        }

        bus.subscribe(OdometerGyro.UpdateOdometerGyroEvent::class){
            val rotation = it.rotate //Angle(_mergeFilter.updateRaw(_gyroRotate.angle, (it.rotate - _gyroRotate).angle))

            bus.invoke(UpdateMergeGyroEvent(rotation, _oldRotation, it.velocity))

            _rotation = rotation
            _velocity = it.velocity

            _oldRotation = rotation

            StaticTelemetry.addData("robot merge rotate", rotation.toDegree())
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
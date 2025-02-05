package org.firstinspires.ftc.teamcode.modules.navigation.odometry

import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.modules.navigation.HardwareOdometers
import org.firstinspires.ftc.teamcode.modules.navigation.gyro.MergeGyro
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Color
import org.firstinspires.ftc.teamcode.utils.units.Vec2
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

class OdometersOdometry : IRobotModule {
    override fun init(collector: BaseCollector, bus: EventBus) {
        _position = collector.parameters.oldStartPosition.position
        _oldRotate = collector.parameters.oldStartPosition.angle

        bus.subscribe(HardwareOdometers.UpdateHardwareOdometersEvent::class) {
            val gyro = bus.invoke(MergeGyro.RequestMergeGyroEvent())

            val deltaLeftPosition = it.leftPosition - it.leftPositionOld
            val deltaRightPosition = it.rightPosition - it.rightPositionOld
            val deltaSidePosition = it.sidePosition - it.sidePositionOld
            val deltaRotate = (gyro.odometerRotate!! - _oldRotate).angle

            val deltaX = if(Configs.OdometryConfig.DUAL_ODOMETER)
                deltaLeftPosition - (Configs.OdometryConfig.FORWARD_ODOMETER_LEFT_RADIUS * deltaRotate)
            else
                (deltaLeftPosition + deltaRightPosition) / 2.0

            val deltaY = deltaSidePosition - (Configs.OdometryConfig.SIDE_ODOMETER_RADIUS * deltaRotate)

            val deltaXCorrected: Double
            val deltaYCorrected: Double

            if(abs(deltaRotate) < Configs.OdometryConfig.ROTATE_SENS) {
                deltaXCorrected = deltaX
                deltaYCorrected = deltaY
            }
            else{
                deltaXCorrected = deltaX * sin(deltaRotate) / deltaRotate + deltaY * (cos(deltaRotate) - 1.0) / deltaRotate
                deltaYCorrected = deltaX * (1.0 - cos(deltaRotate)) / deltaRotate + deltaY * sin(deltaRotate) / deltaRotate
            }

            _position += Vec2(deltaXCorrected, deltaYCorrected
            ).turn(gyro.oldRotation!!.angle)

            _oldRotate = gyro.odometerRotate!!

            val velocity = Vec2(
                if(Configs.OdometryConfig.DUAL_ODOMETER)
                    it.leftVelocity - Configs.OdometryConfig.FORWARD_ODOMETER_LEFT_RADIUS * gyro.velocity!!
                else
                    (it.leftVelocity + it.rightVelocity) / 2.0,
                it.sideVelocity - Configs.OdometryConfig.SIDE_ODOMETER_RADIUS * gyro.velocity!!
            )

            bus.invoke(UpdateOdometersOdometryEvent(_position, velocity))
        }
    }

    private var _position = Vec2.ZERO
    private var _oldRotate = Angle.ZERO

    class UpdateOdometersOdometryEvent(val position: Vec2, val velocity: Vec2): IEvent
}
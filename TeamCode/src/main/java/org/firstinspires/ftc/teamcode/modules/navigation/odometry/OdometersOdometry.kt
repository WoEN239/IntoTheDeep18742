package org.firstinspires.ftc.teamcode.modules.navigation.odometry

import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.modules.navigation.gyro.Gyro
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.motor.EncoderController
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Vec2
import kotlin.math.PI

object OdometersOdometry : IRobotModule {
    private lateinit var _forwardOdometerLeft: EncoderController
    private lateinit var _forwardOdometerRight: EncoderController
    private lateinit var _sideOdometer: EncoderController

    override fun init(collector: BaseCollector) {
        val calc: (Int) -> Double =
            { (it / Configs.OdometryConfig.ODOMETER_TICKS).toDouble() * PI * (Configs.OdometryConfig.ODOMETER_DIAMETER / 2) }

        _forwardOdometerLeft = EncoderController(collector.devices.forwardOdometerLeft, calc)
        _forwardOdometerRight = EncoderController(collector.devices.forwardOdometerRight, calc)
        _sideOdometer = EncoderController(collector.devices.sideOdometer, calc)
    }

    var position = Vec2.ZERO
    var velocity = Vec2.ZERO

    fun calculateRotate() =
        Angle((_forwardOdometerLeft.turnPosition / Configs.OdometryConfig.FORWARD_ODOMETER_LEFT_RADIUS + _forwardOdometerRight.turnPosition / Configs.OdometryConfig.FORWARD_ODOMETER_RIGHT_RADIUS) / 2.0)

    fun calculateRotateVelocity() =
        Angle((_forwardOdometerLeft.velocity / Configs.OdometryConfig.FORWARD_ODOMETER_LEFT_RADIUS + _forwardOdometerRight.velocity / Configs.OdometryConfig.FORWARD_ODOMETER_RIGHT_RADIUS) / 2.0)

    private var _oldForwardOdometerLeft = 0.0
    private var _oldForwardOdometerRight = 0.0
    private var _oldSideOdometer = 0.0
    private var _oldRotate = Angle(0.0)

    override fun update() {
        val deltaForwardOdometerLeft = _forwardOdometerLeft.turnPosition - _oldForwardOdometerLeft
        val deltaForwardOdometerRight = _forwardOdometerRight.turnPosition - _oldForwardOdometerRight
        val deltaSideOdometer = _sideOdometer.turnPosition - _oldSideOdometer
        val deltaRotate = Gyro.rotation - _oldRotate

        position += Vec2(
            (deltaForwardOdometerRight + deltaForwardOdometerLeft) / 2.0,
            deltaSideOdometer - (Angle(Configs.OdometryConfig.SIDE_ODOMETER_RADIUS) * deltaRotate).angle
        ).turn(Gyro.rotation.angle)

        velocity = Vec2(
            (_forwardOdometerLeft.velocity + _forwardOdometerRight.velocity) / 2.0,
            _sideOdometer.velocity - Configs.OdometryConfig.SIDE_ODOMETER_RADIUS * Gyro.velocity
        )

        _oldForwardOdometerLeft = _forwardOdometerLeft.turnPosition
        _oldForwardOdometerRight = _forwardOdometerRight.turnPosition
        _oldSideOdometer = _sideOdometer.turnPosition

        _oldRotate = Gyro.rotation
    }
}
package org.firstinspires.ftc.teamcode.modules.navigation.odometry

import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.modules.navigation.gyro.Gyro
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.motor.EncoderController
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Vec2

object OdometersOdometry : IRobotModule {
    private lateinit var _forwardOdometerLeft: EncoderController
    private lateinit var _forwardOdometerRight: EncoderController
    private lateinit var _sideOdometer: EncoderController

    override fun init(collector: BaseCollector) {
        _forwardOdometerLeft = EncoderController(collector.devices.forwardOdometerLeft)
        _forwardOdometerRight = EncoderController(collector.devices.forwardOdometerRight)
        _sideOdometer = EncoderController(collector.devices.sideOdometer)
    }

    var position = Vec2.ZERO
    var velocity = Vec2.ZERO

    fun calculateRotate() =
        Angle((_forwardOdometerLeft.position / Configs.OdometryConfig.FORWARD_ODOMETER_LEFT_RADIUS + _forwardOdometerRight.position / Configs.OdometryConfig.FORWARD_ODOMETER_RIGHT_RADIUS) / 2.0)

    fun calculateRotateVelocity() =
        Angle((_forwardOdometerLeft.velocity / Configs.OdometryConfig.FORWARD_ODOMETER_LEFT_RADIUS + _forwardOdometerRight.velocity / Configs.OdometryConfig.FORWARD_ODOMETER_RIGHT_RADIUS) / 2.0)

    private var _oldForwardOdometerLeft = 0
    private var _oldForwardOdometerRight = 0
    private var _oldSideOdometer = 0
    private var _oldRotate = Angle(0.0)

    override fun update() {
        val deltaForwardOdometerLeft = _forwardOdometerLeft.position - _oldForwardOdometerLeft
        val deltaForwardOdometerRight =  _forwardOdometerRight.position - _oldForwardOdometerRight
        val deltaSideOdometer = _sideOdometer.position - _oldSideOdometer
        val deltaRotate = Gyro.rotation - _oldRotate

        position += Vec2((deltaForwardOdometerRight + deltaForwardOdometerLeft) / 2.0,
            deltaSideOdometer - (Angle(Configs.OdometryConfig.SIDE_ODOMETER_RADIUS) * deltaRotate).angle).turn(Gyro.rotation.angle)

        velocity = Vec2((_forwardOdometerLeft.velocity + _forwardOdometerRight.velocity) / 2.0,
            _sideOdometer.velocity - Configs.OdometryConfig.SIDE_ODOMETER_RADIUS * Gyro.velocity)

        _oldForwardOdometerLeft = _forwardOdometerLeft.position
        _oldForwardOdometerRight = _forwardOdometerRight.position
        _oldSideOdometer = _sideOdometer.position

        _oldRotate = Gyro.rotation
    }
}
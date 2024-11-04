package org.firstinspires.ftc.teamcode.modules.navigation.odometry

import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.motor.EncoderFix
import kotlin.math.PI

class HardwareOdometers : IRobotModule {
    private lateinit var _forwardOdometerLeft: EncoderFix
    private lateinit var _forwardOdometerRight: EncoderFix
    private lateinit var _sideOdometer: EncoderFix

    private lateinit var _eventBus: EventBus

    override fun init(collector: BaseCollector, bus: EventBus) {
        val calc: (Double) -> Double =
            { (it / Configs.OdometryConfig.ODOMETER_TICKS) * PI * Configs.OdometryConfig.ODOMETER_DIAMETER }

        _forwardOdometerLeft = EncoderFix(collector.devices.forwardOdometerLeft, calc)
        _forwardOdometerRight = EncoderFix(collector.devices.forwardOdometerRight, calc)

        _sideOdometer = EncoderFix(collector.devices.sideOdometer, calc)

        _eventBus = bus
    }

    private var _oldPositionLeft: Double = 0.0
    private var _oldPositionRight: Double = 0.0
    private var _oldPositionSide: Double = 0.0

    override fun update() {
        val currentLeftPosition = _forwardOdometerLeft.realPosition
        val currentRightPosition = _forwardOdometerRight.realPosition
        val currentSidePosition = _sideOdometer.realPosition

        _eventBus.invoke(
            UpdateHardwareOdometersEvent(
                currentLeftPosition, currentRightPosition, currentSidePosition,
                _oldPositionLeft, _oldPositionRight, _oldPositionSide,
                _forwardOdometerLeft.realVelocity, _forwardOdometerRight.realVelocity, _sideOdometer.realVelocity
            ))

        _oldPositionLeft = currentLeftPosition
        _oldPositionRight = currentRightPosition
        _oldPositionSide = currentSidePosition
    }

    override fun start() {
        _oldPositionLeft = _forwardOdometerLeft.realPosition
        _oldPositionRight = _forwardOdometerRight.realPosition
        _oldPositionSide = _sideOdometer.realPosition
    }

    class UpdateHardwareOdometersEvent(
        val leftPosition: Double, val rightPosition: Double, val sidePosition: Double,
        val leftPositionOld: Double, val rightPositionOld: Double, val sidePositionOld: Double,
        val leftVelocity: Double, val rightVelocity: Double, val sideVelocity: Double
    ): IEvent
}
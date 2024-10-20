package org.firstinspires.ftc.teamcode.modules.navigation.odometry

import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.motor.EncoderFix
import kotlin.math.PI

object HardwareOdometers : IRobotModule {
    private lateinit var _forwardOdometerLeft: EncoderFix
    private lateinit var _forwardOdometerRight: EncoderFix
    private lateinit var _sideOdometer: EncoderFix

    override fun init(collector: BaseCollector) {
        val calc: (Int) -> Double =
            { (it.toDouble() / Configs.OdometryConfig.ODOMETER_TICKS) * PI * Configs.OdometryConfig.ODOMETER_DIAMETER }

        _forwardOdometerLeft = EncoderFix(collector.devices.forwardOdometerLeft, calc)
        _forwardOdometerRight = EncoderFix(collector.devices.forwardOdometerRight, calc)

        _forwardOdometerRight.encoder.direction = DcMotorSimple.Direction.REVERSE

        _sideOdometer = EncoderFix(collector.devices.sideOdometer, calc)
    }

    var oldPositionForwardOdometerLeft: Double = 0.0
        private set

    var oldPositionForwardOdometerRight: Double = 0.0
        private set

    var oldPositionSideOdometer: Double = 0.0
        private set

    var forwardOdometerLeftPosition: Double = 0.0
        private set

    var forwardOdometerRightPosition: Double = 0.0
        private set

    var sideOdometerPosition: Double = 0.0
        private set

    val forwardOdometerLeftVelocity: Double
        get() = _forwardOdometerLeft.velocity

    val forwardOdometerRightVelocity: Double
        get() = _forwardOdometerRight.velocity

    val sideOdometerVelocity: Double
        get() = _sideOdometer.velocity

    override fun update() {
        oldPositionForwardOdometerLeft = forwardOdometerLeftPosition
        oldPositionForwardOdometerRight = forwardOdometerRightPosition
        oldPositionSideOdometer = sideOdometerPosition

        forwardOdometerLeftPosition = _forwardOdometerLeft.calcPos
        forwardOdometerRightPosition = _forwardOdometerRight.calcPos
        sideOdometerPosition = _sideOdometer.calcPos
    }
}
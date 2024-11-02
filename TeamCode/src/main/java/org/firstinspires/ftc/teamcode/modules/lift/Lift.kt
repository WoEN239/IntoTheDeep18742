package org.firstinspires.ftc.teamcode.modules.lift

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.pidRegulator.PIDRegulator
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry

object Lift : IRobotModule {
    var targetPosition = LiftPosition.DOWN

    var powerUp = 0.0

    override fun update() {
        val encLeft = _motorLeft.currentPosition.toDouble() - _softResetPositionLeft
        val encRight = _motorRight.currentPosition.toDouble() - _softResetPositionRight

        val powerLeft: Double
        val powerRight: Double

        /*StaticTelemetry.addLine("targetPosition = $targetPosition")

        if (targetPosition != LiftPosition.DOWN) {
            val power =p
                _posPID.update(
                    (if (targetPosition == LiftPosition.MIDDLE) Configs.LiftConfig.LIFT_MIDDLE_POS else Configs.LiftConfig.LIFT_UP_POS)
                            - (encLeft + encRight) / 2.0
                )

            powerRight = power
            powerLeft = power
        }
        else {
            powerLeft = if (!_endingLeft.state) Configs.LiftConfig.DOWN_SPEED else Configs.LiftConfig.DOWN_SPEEDLOW
            powerRight = if (!_endingRight.state) Configs.LiftConfig.DOWN_SPEED else Configs.LiftConfig.DOWN_SPEEDLOW

            if(_endingLeft.state)
                _softResetPositionLeft = _motorLeft.currentPosition

            if(_endingRight.state)
                _softResetPositionRight = _motorRight.currentPosition
        }*/

        val uEnc = _syncPID.update(encLeft - encRight)

        _motorLeft.power = powerUp + uEnc
        _motorRight.power = powerUp - uEnc
    }

    private var _softResetPositionLeft = 0
    private var _softResetPositionRight = 0

    private lateinit var _motorLeft: DcMotor
    private lateinit var _motorRight: DcMotor

    private lateinit var _endingRight: DigitalChannel  //goida
    private lateinit var _endingLeft: DigitalChannel

    private val _posPID = PIDRegulator(Configs.LiftConfig.LIFT_PID)
    private val _syncPID = PIDRegulator(Configs.LiftConfig.LIFT_PID_SYNC)

    val isDown: Boolean
        get() = (_motorLeft.currentPosition + _motorRight.currentPosition) / 2.0 < Configs.LiftConfig.LIFT_DOWN_POS

    override fun init(collector: BaseCollector) {
        _motorLeft = collector.devices.liftMotorLeft
        _motorRight = collector.devices.liftMotorRight

        _motorLeft.direction = DcMotorSimple.Direction.REVERSE

        _motorLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        _motorRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        _endingLeft = collector.devices.liftEndingLeft
        _endingRight = collector.devices.liftEndingRight
    }

    override fun start() {
        _motorLeft.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        _motorRight.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        _motorLeft.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        _motorRight.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    enum class LiftPosition { UP, DOWN, MIDDLE }
}
package org.firstinspires.ftc.teamcode.modules.lift

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.pidRegulator.PIDRegulator

object Lift : IRobotModule {
    var targetPosition = LiftPosition.DOWN

    override fun update() {
        var power = 0.0

        val encLeft = _motorLeft.currentPosition.toDouble()
        val encRight = _motorRight.currentPosition.toDouble()

        if (targetPosition == LiftPosition.MIDDLE)
            power =
                _posPID.update(
                    (if (targetPosition == LiftPosition.MIDDLE) Configs.LiftConfig.LIFT_MIDDLE_POS else Configs.LiftConfig.LIFT_UP_POS)
                            - (encLeft + encRight) / 2.0)
        else
            power =
                if (_endingDown.state) Configs.LiftConfig.DOWN_SPEED else Configs.LiftConfig.DOWN_SPEEDLOW

        val uEnc = _syncPID.update(encLeft - encRight)

        _motorLeft.power = power - uEnc
        _motorRight.power = power + uEnc
    }

    private lateinit var _motorLeft: DcMotor
    private lateinit var _motorRight: DcMotor

    private lateinit var _endingDown: DigitalChannel
    private lateinit var _endingUp: DigitalChannel

    private val _posPID = PIDRegulator(Configs.LiftConfig.LIFT_PID)
    private val _syncPID = PIDRegulator(Configs.LiftConfig.LIFT_PID_SYNC)

    override fun init(collector: BaseCollector) {
        _motorLeft = collector.devices.liftMotorLeft
        _motorRight = collector.devices.liftMotorRight

        _motorRight.direction = DcMotorSimple.Direction.REVERSE

        _endingUp = collector.devices.endingUP
        _endingDown = collector.devices.endingDown
    }

    override fun start() {
        _motorLeft.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        _motorRight.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        _motorLeft.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        _motorRight.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    enum class LiftPosition { UP, DOWN, MIDDLE }
}
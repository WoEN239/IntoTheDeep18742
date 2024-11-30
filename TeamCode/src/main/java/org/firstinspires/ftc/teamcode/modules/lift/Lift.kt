package org.firstinspires.ftc.teamcode.modules.lift

import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
import com.qualcomm.robotcore.hardware.DigitalChannel
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.devices.Battery
import org.firstinspires.ftc.teamcode.utils.pidRegulator.PIDRegulator
import kotlin.math.PI
import kotlin.math.abs

class Lift: IRobotModule {
    class SetLiftTargetEvent(val targetAimPos: Double, val targetExtensionPos: Double): IEvent
    class RequestLiftAtTargetEvent(var atTarget: Boolean): IEvent
    class RequestCurrentLiftPosition(var aimPos: Double = 0.0, var extensionPos: Double = 0.0): IEvent
    class RequestCurrentLiftTarget(var aimPos: Double = 0.0, var extensionPos: Double = 0.0): IEvent

    private lateinit var _aimMotor: DcMotorEx
    private lateinit var _extensionMotor: DcMotorEx

    private lateinit var _aimEndingUp: DigitalChannel
    private lateinit var _extensionEndingDown: DigitalChannel

    private var _targetAimPos = 0.0
    private var _targetExtensionPos = 0.0

    private val _aimPID = PIDRegulator(Configs.LiftConfig.AIM_PID)
    private val _extensionPID = PIDRegulator(Configs.LiftConfig.EXTENSION_PID)

    private lateinit var _battery: Battery

    private var _aimErr = 0.0
    private var _promotedErr = 0.0

    private var _aimStartPosition = 0
    private var _extensionStartPosition = 0

    override fun init(collector: BaseCollector, bus: EventBus) {
        _battery = collector.devices.battery

        _aimMotor = collector.devices.liftAimMotor
        _extensionMotor = collector.devices.liftExtensionMotor

        _extensionMotor.direction = REVERSE

        _aimMotor.zeroPowerBehavior = BRAKE
        _extensionMotor.zeroPowerBehavior = BRAKE

        _aimMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        _aimMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        _extensionMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        _extensionMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        _aimEndingUp = collector.devices.liftAimEndingUp
        _extensionEndingDown = collector.devices.liftExtensionEndingDown

        bus.subscribe(SetLiftTargetEvent::class){
            _targetAimPos = clamp(it.targetAimPos, Configs.LiftConfig.MIN_AIM_POS, Configs.LiftConfig.MAX_AIM_POS)
            _targetExtensionPos = clamp(it.targetExtensionPos, Configs.LiftConfig.MIN_EXTENSION_POS, Configs.LiftConfig.MAX_EXTENSION_POS)
        }

        bus.subscribe(RequestLiftAtTargetEvent::class){
            it.atTarget = abs(_aimErr) < Configs.LiftConfig.AIM_SENS && abs(_promotedErr) < Configs.LiftConfig.PROMOTED_SENS
        }

        bus.subscribe(RequestCurrentLiftPosition::class){
            it.aimPos = getCurrentAimPos()
            it.extensionPos = getCurrentExtensionPos()
        }

        bus.subscribe(RequestCurrentLiftTarget::class){
            it.aimPos = _targetAimPos
            it.extensionPos = _targetExtensionPos
        }
    }

    fun getCurrentAimPos() = (_aimMotor.currentPosition - _aimStartPosition).toDouble()
    fun getCurrentExtensionPos() = (_extensionMotor.currentPosition - _extensionStartPosition).toDouble()

    override fun update() {
        if(_aimEndingUp.state)
            _aimStartPosition = _aimMotor.currentPosition - Configs.LiftConfig.LIFT_ENDING_POS

        if(_extensionEndingDown.state)
            _extensionStartPosition = _extensionMotor.currentPosition

        _aimErr = _targetAimPos - getCurrentAimPos()
        _promotedErr = _targetExtensionPos - getCurrentExtensionPos()

        val aimPower = _aimPID.update(_aimErr).coerceAtLeast(-abs(Configs.LiftConfig.MAX_SPEED_DOWN)) / _battery.charge
        val extensionPower = _extensionPID.update(_promotedErr) / _battery.charge

        _aimMotor.power = aimPower
        _extensionMotor.power = -extensionPower
    }

    override fun start() {

    }
}
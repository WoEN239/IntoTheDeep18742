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
import org.firstinspires.ftc.teamcode.utils.configs.Configs.LiftConfig.LiftPosition
import org.firstinspires.ftc.teamcode.utils.devices.Battery
import org.firstinspires.ftc.teamcode.utils.pidRegulator.PIDRegulator
import kotlin.math.PI
import kotlin.math.abs

class Lift: IRobotModule {
    class RequestLiftAtTargetEvent(var atTarget: Boolean): IEvent
    class SetLiftStateEvent(val state: LiftStates): IEvent

    private lateinit var _aimMotor: DcMotorEx
    private lateinit var _extensionMotor: DcMotorEx

    private lateinit var _aimEndingUp: DigitalChannel
    private lateinit var _extensionEndingDown: DigitalChannel

    private var _targetAimPos = 100.0
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

        bus.subscribe(RequestLiftAtTargetEvent::class){
            it.atTarget = abs(_aimErr) < Configs.LiftConfig.AIM_SENS && abs(_promotedErr) < Configs.LiftConfig.PROMOTED_SENS
        }

        _aimMotor.power = abs(Configs.LiftConfig.INIT_POWER)

        bus.subscribe(SetLiftStateEvent::class){
            updateLiftState(it.state)
        }

        updateLiftState(LiftStates.SETUP)
    }

    fun updateLiftState(state: LiftStates){
        fun setLiftTarget(target: LiftPosition){
            _targetAimPos = target.AIM_POSITION
            _targetExtensionPos = target.EXTENSION_POSITION
        }

        when(state){
            LiftStates.UP_BASKET -> setLiftTarget(Configs.LiftConfig.TARGET_UP_BASKET_LIFT_POSITION)
            LiftStates.MIDDLE_BASKET -> setLiftTarget(Configs.LiftConfig.TARGET_MIDDLE_BASKET_LIFT_POSITION)
            LiftStates.DOWN_BASKET -> setLiftTarget(Configs.LiftConfig.TARGET_DOWN_BASKET_LIFT_POSITION)
            LiftStates.UP_LAYER -> setLiftTarget(Configs.LiftConfig.TARGET_UP_LAYER_LIFT_POSITION)
            LiftStates.DOWN_LAYER -> setLiftTarget(Configs.LiftConfig.TARGET_DOWN_LAYER_LIFT_POSITION)
            LiftStates.CLAMP_FIELD -> setLiftTarget(Configs.LiftConfig.TARGET_CLAMP_FIELD_LIFT_POSITION)
            LiftStates.CLAMP_CENTER -> setLiftTarget(Configs.LiftConfig.TARGET_CLAMP_CENTER_LIFT_POSITION)
            LiftStates.CLAMP_WALL -> setLiftTarget(Configs.LiftConfig.TARGET_CLAMP_WALL_LIFT_POSITION)
            LiftStates.SETUP -> setLiftTarget(Configs.LiftConfig.TARGET_SETUP_LIFT_POSITION)
        }
    }

    fun getCurrentAimPos() = (_aimMotor.currentPosition - _aimStartPosition).toDouble()
    fun getCurrentExtensionPos() = (_extensionMotor.currentPosition - _extensionStartPosition).toDouble()

    override fun update() {
        if(!_aimEndingUp.state)
            _aimStartPosition = _aimMotor.currentPosition - Configs.LiftConfig.LIFT_ENDING_POS

        if(!_extensionEndingDown.state)
            _extensionStartPosition = _extensionMotor.currentPosition

        _aimErr = _targetAimPos - getCurrentAimPos()
        _promotedErr = (_targetExtensionPos + getCurrentAimPos() * Configs.LiftConfig.EXTENSION_FIX) - getCurrentExtensionPos()

        val aimPower = _aimPID.update(_aimErr).coerceAtLeast(Configs.LiftConfig.MAX_SPEED_DOWN) / _battery.charge
        val extensionPower = _extensionPID.update(_promotedErr) / _battery.charge

        _aimMotor.power = aimPower
        _extensionMotor.power = extensionPower
    }

    enum class LiftStates{
        UP_BASKET,
        MIDDLE_BASKET,
        DOWN_BASKET,
        UP_LAYER,
        DOWN_LAYER,
        CLAMP_FIELD,
        CLAMP_CENTER,
        CLAMP_WALL,
        SETUP
    }

    override fun start() {

    }
}
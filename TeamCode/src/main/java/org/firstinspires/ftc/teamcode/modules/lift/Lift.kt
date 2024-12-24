package org.firstinspires.ftc.teamcode.modules.lift

import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.configs.Configs.LiftConfig.LiftPosition
import org.firstinspires.ftc.teamcode.utils.devices.Battery
import org.firstinspires.ftc.teamcode.utils.pidRegulator.PIDRegulator
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.timer.Timers
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos

class Lift: IRobotModule {
    class RequestLiftAtTargetEvent(var atTarget: Boolean): IEvent
    class RequestLiftState(var state: LiftStates? = null): IEvent
    class SetLiftStateEvent(val state: LiftStates): IEvent
    class SetExtensionVelocityEvent(val velocity: Double): IEvent
    class LiftStateSwap(val state: LiftStates): IEvent

    private lateinit var _aimMotor: DcMotorEx
    private lateinit var _extensionMotor: DcMotorEx

    private lateinit var _aimEndingUp: DigitalChannel
    private lateinit var _aimEndingDown: DigitalChannel
    private lateinit var _extensionEndingDown: DigitalChannel

    private val _aimPID = PIDRegulator(Configs.LiftConfig.AIM_PID)
    private val _extensionPID = PIDRegulator(Configs.LiftConfig.EXTENSION_PID)

    private lateinit var _battery: Battery

    private var _aimErr = 0.0
    private var _extensionErr = 0.0

    private var _aimStartPosition = 0
    private var _extensionStartPosition = 0

    private var _extensionVelocity = 0.0

    private var _targetState = LiftStates.SETUP

    private lateinit var _eventBus: EventBus

    override fun init(collector: BaseCollector, bus: EventBus) {
        _eventBus = bus;

        _battery = collector.devices.battery

        _aimMotor = collector.devices.liftAimMotor
        _extensionMotor = collector.devices.liftExtensionMotor

        _extensionMotor.direction = REVERSE

        _aimMotor.zeroPowerBehavior = BRAKE
        _extensionMotor.zeroPowerBehavior = BRAKE

        if(collector.gameSettings.isAuto) {
            _aimMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            _aimMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

            _extensionMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            _extensionMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }

        _aimEndingUp = collector.devices.liftAimEndingUp
        _extensionEndingDown = collector.devices.liftExtensionEndingDown
        _aimEndingDown = collector.devices.liftAimEndingDown

        bus.subscribe(RequestLiftAtTargetEvent::class){
            it.atTarget = abs(_aimErr) < Configs.LiftConfig.AIM_SENS && abs(_extensionErr) < Configs.LiftConfig.EXTENSION_SENS
        }

        _aimMotor.power = Configs.LiftConfig.INIT_POWER

        bus.subscribe(SetLiftStateEvent::class){
            val clampPose = bus.invoke(Intake.RequestClampStateEvent()).state!!

            if(it.state == LiftStates.SETUP ||
                ((it.state == LiftStates.CLAMP_CENTER || it.state == LiftStates.CLAMP_WALL) && clampPose == Intake.ClampPosition.SERVO_UNCLAMP) || ((it.state == LiftStates.UP_BASKET || it.state == LiftStates.UP_LAYER) && clampPose == Intake.ClampPosition.SERVO_CLAMP)
            ) {
                _targetState = it.state

                _deltaExtension = 0.0
                _extensionVelocity = 0.0

                bus.invoke(LiftStateSwap(it.state))
            }
        }

        _targetState = LiftStates.SETUP

        bus.subscribe(SetExtensionVelocityEvent::class){
            if(_targetState == LiftStates.CLAMP_CENTER)
                _extensionVelocity = it.velocity
        }

        bus.subscribe(Intake.SetClampStateEvent::class){
            if(_targetState != LiftStates.SETUP)
                Timers.newTimer().start(Configs.LiftConfig.CLAMP_TIME){
                    bus.invoke(SetLiftStateEvent(LiftStates.SETUP))
                }
        }

        bus.subscribe(RequestLiftState::class){
            it.state = _targetState
        }
    }

    fun getTargetsByState(state: LiftStates): Pair<Double, Double> {
        fun target(pos: LiftPosition) = Pair(pos.AIM_POSITION, pos.EXTENSION_POSITION)

        return when(state){
            LiftStates.UP_BASKET -> target(Configs.LiftConfig.TARGET_UP_BASKET_LIFT_POSITION)
            LiftStates.UP_LAYER -> target(Configs.LiftConfig.TARGET_UP_LAYER_LIFT_POSITION)
            LiftStates.CLAMP_CENTER -> target(Configs.LiftConfig.TARGET_CLAMP_CENTER_LIFT_POSITION)
            LiftStates.CLAMP_WALL -> target(Configs.LiftConfig.TARGET_CLAMP_WALL_LIFT_POSITION)
            LiftStates.SETUP -> target(Configs.LiftConfig.TARGET_SETUP_LIFT_POSITION)
        }
    }

    fun getCurrentAimPos() = (_aimMotor.currentPosition - _aimStartPosition).toDouble()
    fun getCurrentExtensionPos() = (_extensionMotor.currentPosition - _extensionStartPosition).toDouble()

    private val _deltaTime = ElapsedTime()

    private var _deltaExtension = 0.0
    private var _oldTargetAimPos = 0.0

    override fun update() {
        _deltaExtension += _deltaTime.seconds() * _extensionVelocity

        _deltaExtension = clamp(
            _deltaExtension,
            Configs.LiftConfig.MIN_EXTENSION_POS,
            Configs.LiftConfig.MAX_EXTENSION_POS
        )

        _deltaTime.reset()

        val targets = getTargetsByState(_targetState)

        val clampState = _eventBus.invoke(Intake.RequestClampStateEvent()).state

        val targetExtensionPos = targets.second + _deltaExtension
        val targetAimPos = targets.first //-
                //(if(clampState == Intake.ClampPosition.SERVO_CLAMP && _targetState == LiftStates.CLAMP_CENTER) 50.0 else 0.0)

        if (_aimEndingUp.state)
            _aimStartPosition = _aimMotor.currentPosition - Configs.LiftConfig.LIFT_AIM_ENDING_POS

        if (_aimEndingDown.state)
            _aimStartPosition = _aimMotor.currentPosition

        if(_extensionEndingDown.state)
            _extensionStartPosition = _extensionMotor.currentPosition

        val targetDefencedAimPos: Double

        if (abs(Configs.LiftConfig.MIN_EXTENSION_POS - getCurrentExtensionPos()) < Configs.LiftConfig.EXTENSION_SENS) {
            targetDefencedAimPos = targetAimPos
            _oldTargetAimPos = targetAimPos
        }
        else
            targetDefencedAimPos = _oldTargetAimPos

        val targetDefencedExtensionPos: Double

        if(abs(targetAimPos - getCurrentAimPos()) > Configs.LiftConfig.AIM_SENS)
            targetDefencedExtensionPos = Configs.LiftConfig.MIN_EXTENSION_POS
        else
            targetDefencedExtensionPos = targetExtensionPos

        _aimErr = targetDefencedAimPos - getCurrentAimPos()
        _extensionErr = (targetDefencedExtensionPos + ((getCurrentAimPos() / Configs.LiftConfig.AIM_TURN_TICKS) * Configs.LiftConfig.EXTENSION_FIX) / Configs.LiftConfig.EXTENSION_TURN_TICKS) - getCurrentExtensionPos()

        val triggerMinPower = if (getCurrentAimPos() > Configs.LiftConfig.TRIGET_SLOW_POS) Configs.LiftConfig.MAX_SPEED_DOWN
        else Configs.LiftConfig.MAX_TRIGGER_SPEED_DOWN

        val aimPower = (_aimPID.update(_aimErr)
            .coerceAtLeast(triggerMinPower).coerceAtMost(Configs.LiftConfig.MIN_SPEED_UP)) /
                _battery.charge

        val extensionPower = _extensionPID.update(_extensionErr) / _battery.charge

        _aimMotor.power = aimPower
        _extensionMotor.power = extensionPower
    }

    enum class LiftStates{
        UP_BASKET,
        UP_LAYER,
        CLAMP_CENTER,
        CLAMP_WALL,
        SETUP
    }

    override fun start() {

    }
}
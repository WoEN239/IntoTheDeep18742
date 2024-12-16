package org.firstinspires.ftc.teamcode.modules.lift

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
import com.qualcomm.robotcore.hardware.DigitalChannel
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.devices.Battery
import org.firstinspires.ftc.teamcode.utils.pidRegulator.PIDRegulator
import org.firstinspires.ftc.teamcode.utils.timer.Timers
import kotlin.math.abs

class Lift: IRobotModule {
    private lateinit var _aimMotor: DcMotorEx
    private lateinit var _extensionMotor: DcMotorEx

    private lateinit var _aimEndingUp: DigitalChannel
    private lateinit var _extensionEndingDown: DigitalChannel
    private lateinit var _battery: Battery
    class RequestLiftAtTargetEvent(var atTarget: Boolean): IEvent
    class RequestLiftState(var state: LiftStates? = null): IEvent
    class SetLiftStateEvent(val state: LiftStates): IEvent
    class SetExtensionVelocityEvent(val velocity: Double): IEvent
    class LiftStateSwap(val state: LiftStates): IEvent

    private val _aimPID = PIDRegulator(Configs.LiftConfig.AIM_PID)
    private val _extensionPID = PIDRegulator(Configs.LiftConfig.EXTENSION_PID)
    private var _aimErr = 0.0
    private var _extensionErr = 0.0
    var _targetState = LiftStates.SETUP

    override fun init(collector: BaseCollector, bus: EventBus) {
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
        } // обнуление энкодеров в начале автонома

        _aimEndingUp = collector.devices.liftAimEndingUp
        _extensionEndingDown = collector.devices.liftExtensionEndingDown

        bus.subscribe(SetLiftStateEvent::class)
        {
            val clampPosition = bus.invoke(Intake.RequestClampPositionEvent()).position


            if(_targetState == LiftStates.SETUP && clampPosition == Intake.ClampPosition.SERVO_CLAMP && it.state == LiftStates.UP_BASKET)
                _targetState = it.state
            if(_targetState == LiftStates.SETUP && clampPosition == Intake.ClampPosition.SERVO_UNCLAMP && it.state == LiftStates.CLAMP_CENTER)
                _targetState = it.state
            if(_targetState == LiftStates.SETUP && clampPosition == Intake.ClampPosition.SERVO_CLAMP && it.state == LiftStates.UP_LAYER)
                _targetState = it.state
            if(_targetState == LiftStates.SETUP && clampPosition == Intake.ClampPosition.SERVO_UNCLAMP && it.state == LiftStates.CLAMP_DOWN)
                _targetState = it.state
            if(_targetState == LiftStates.SETUP && clampPosition == Intake.ClampPosition.SERVO_UNCLAMP && it.state == LiftStates.CLAMP_WALL_DOWN1)
                _targetState = it.state
        }
        bus.subscribe(Intake.SetClampEvent::class)
        {
            Timers.newTimer().start(Configs.LiftConfig.Time_Center)
            {
                _targetState = LiftStates.SETUP
            }
        }

        bus.subscribe(RequestLiftAtTargetEvent::class){

            it.atTarget = abs(_aimErr) < Configs.LiftConfig.AIM_SENS && abs(_extensionErr) < Configs.LiftConfig.EXTENSION_SENS
        }
    }
    fun getTargetsByState(state: LiftStates): Pair<Double, Double> {
        fun target(pos: Configs.LiftConfig.LiftPosition) = Pair(pos.AIM_POSITION, pos.EXTENSION_POSITION)

        return when(state){
            LiftStates.UP_BASKET -> target(Configs.LiftConfig.TARGET_UP_BASKET_LIFT_POSITION)
            LiftStates.UP_LAYER -> target(Configs.LiftConfig.TARGET_UP_LAYER_LIFT_POSITION)
            LiftStates.CLAMP_CENTER -> target(Configs.LiftConfig.TARGET_CLAMP_CENTER_LIFT_POSITION)
            LiftStates.CLAMP_DOWN -> target(Configs.LiftConfig.TARGET_CLAMP_DOWN_LIFT_POSITION)
            LiftStates.SETUP -> target(Configs.LiftConfig.TARGET_SETUP_LIFT_POSITION)
            LiftStates.CLAMP_WALL_DOWN1 -> target(Configs.LiftConfig.Target_CLAMP_WALL_DOWN1_POSITION)
            LiftStates.CLAMP_WALL_DOWN2 -> target(Configs.LiftConfig.Target_CLAMP_WALL_DOWN2_POSITION)
        }
    }
    override fun update() {
        val targets = getTargetsByState(_targetState)
        _extensionErr = _extensionMotor.currentPosition.toDouble()
        if(abs(_extensionMotor.currentPosition.toDouble()) > Configs.LiftConfig.EXTENSION_SENS && _targetState != LiftStates.SETUP && _targetState != LiftStates.CLAMP_WALL_DOWN2) {
            _extensionMotor.power = _extensionPID.update(_extensionErr)
        }
        _aimErr = targets.first - _aimMotor.currentPosition
        _aimMotor.power = _aimPID.update(_aimErr)
        _extensionErr = targets.second - _extensionMotor.currentPosition
        _extensionMotor.power = _extensionPID.update(_extensionErr)
    }
    enum class LiftStates{
        UP_BASKET,
        UP_LAYER,
        CLAMP_CENTER,
        CLAMP_DOWN,
        SETUP,
        CLAMP_WALL_DOWN1,
        CLAMP_WALL_DOWN2
    }
    override fun start() {

    }
}
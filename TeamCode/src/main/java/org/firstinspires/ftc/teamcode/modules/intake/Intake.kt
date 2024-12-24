package org.firstinspires.ftc.teamcode.modules.intake

import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.modules.lift.Lift
import org.firstinspires.ftc.teamcode.modules.lift.Lift.LiftStateSwap
import org.firstinspires.ftc.teamcode.modules.lift.Lift.RequestLiftAtTargetEvent
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.timer.Timers

class Intake() : IRobotModule {
    class SetClampStateEvent(val state: ClampPosition): IEvent
    class SetDifVelocityEvent(val xVel: Double, val yVel: Double): IEvent
    class RequestClampStateEvent(var state: ClampPosition? = null): IEvent

    private lateinit var _servoClamp: Servo

    private lateinit var _servoDifleft: Servo
    private lateinit var _servoDifRight: Servo

    private var _xVelocity = 0.0
    private var _yVelocity = 0.0

    private var _xPos = 0.0
    private var _yPos = 0.0

    private val _deltaTime = ElapsedTime()

    private var _currentState: Lift.LiftStates = Lift.LiftStates.SETUP

    override fun init(collector: BaseCollector, bus: EventBus) {
        _servoClamp = collector.devices.servoClamp
        _servoDifleft = collector.devices.servoDifLeft
        _servoDifRight = collector.devices.servoDifRight

        bus.subscribe(SetClampStateEvent::class){
            clamp = it.state
        }

        bus.subscribe(SetDifVelocityEvent::class){
            if(_currentState == Lift.LiftStates.CLAMP_CENTER)
                _yVelocity = it.yVel
        }

        bus.subscribe(LiftStateSwap::class){
            if(_currentState == Lift.LiftStates.UP_BASKET && it.state == Lift.LiftStates.SETUP)
                setDifPos(10.0, 0.0)

            _currentState = it.state

            Timers.newTimer().start(Configs.IntakeConfig.LIFT_TIME) {
                if (_currentState == Lift.LiftStates.CLAMP_CENTER)
                    setDifPos(118.0, 0.0)
                else if (_currentState == Lift.LiftStates.UP_BASKET)
                    setDifPos(-10.0, 0.0)
                else if (_currentState == Lift.LiftStates.UP_LAYER)
                    setDifPos(10.0, 0.0)
                else if (_currentState == Lift.LiftStates.SETUP)
                    setDifPos(-70.0, 0.0)
                else
                    setDifPos(0.0, 0.0)
            }
        }

        bus.subscribe(RequestClampStateEvent::class){
            it.state = clamp
        }
    }

    var clamp = ClampPosition.SERVO_UNCLAMP
        set(value) {
            if (value == ClampPosition.SERVO_CLAMP) {
                _servoClamp.position = Configs.IntakeConfig.SERVO_CLAMP
            } else if (value == ClampPosition.SERVO_UNCLAMP) {
                _servoClamp.position = Configs.IntakeConfig.SERVO_UNCLAMP
            }

            field = value
        }


    fun setDifPos(xRot: Double, yRot: Double)
    {
        _xPos = xRot
        _yPos = yRot

        val x = xRot + 135.0
        val y = yRot + 10.0

        _servoDifRight.position = clamp((y + x) / Configs.IntakeConfig.MAX, 0.0, 1.0)
        _servoDifleft.position = clamp(1.0 - (x - y) / Configs.IntakeConfig.MAX, 0.0, 1.0)
    }

    enum class ClampPosition
    {
        SERVO_CLAMP,
        SERVO_UNCLAMP
    }

    override fun update() {
        if(_currentState == Lift.LiftStates.CLAMP_CENTER)
            setDifPos(clamp(_xPos + _deltaTime.seconds() * _xVelocity, -90.0, 90.0), clamp(_yPos + _deltaTime.seconds() * _yVelocity, -90.0, 90.0))

        _deltaTime.reset()
    }

    override fun start() {
        _deltaTime.reset()
        setDifPos(-90.0, 0.0)
        clamp = ClampPosition.SERVO_CLAMP
    }
}
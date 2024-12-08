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
    class SetClampEvent(val position: ClampPosition): IEvent
    class SetDifPosEvent(val yRot: Double, val xRot: Double): IEvent
    class RequestClampPositionEvent(var position: ClampPosition? = null): IEvent

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

        bus.subscribe(SetClampEvent::class){
            clamp = it.position
        }

        bus.subscribe(SetDifPosEvent::class){
            //тут тоже прорверки

            if(_currentState == Lift.LiftStates.CLAMP_CENTER)
                _yVelocity = it.yRot
        }

        bus.subscribe(LiftStateSwap::class){
            _currentState = it.state

            if(_currentState == Lift.LiftStates.CLAMP_CENTER) {
                Timers.newTimer().start({ bus.invoke(RequestLiftAtTargetEvent(false)).atTarget }) {
                    setDifPos(0.0, -60.0)
                }
            }
            else if(_currentState == Lift.LiftStates.UP_BASKET)
                setDifPos(0.0, 40.0)
            else if(_currentState == Lift.LiftStates.UP_LAYER)
                setDifPos(0.0, 10.0)
            else
                setDifPos(0.0, 0.0)

            //тут проверки

            /*if(_currentState == Lift.LiftStates.SETUP)
                setDifPos(90.0, 90.0)

            if(_currentState == Lift.LiftStates.UP_BASKET)
                setDifPos(90.0, 90.0)

            if(_currentState == Lift.LiftStates.UP_LAYER)
                setDifPos(90.0, 90.0)

            if(_currentState == Lift.LiftStates.CLAMP_WALL)
                setDifPos(90.0, 90.0)

            if(_currentState == Lift.LiftStates.CLAMP_CENTER)
                setDifPos(90.0, 90.0)*/
        }

        bus.subscribe(RequestClampPositionEvent::class){
            it.position = clamp
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


    fun setDifPos(yRot: Double,xRot: Double)
    {
        _xPos = xRot
        _yPos = yRot

        val x = xRot + 150.0
        val y = yRot + 17.0

        _servoDifRight.position = clamp((x + y) / Configs.IntakeConfig.MAX, 0.0, 1.0)
        _servoDifleft.position = clamp(1.0 - (x - y) / Configs.IntakeConfig.MAX, 0.0, 1.0)
    }

    enum class ClampPosition
    {
        SERVO_CLAMP,
        SERVO_UNCLAMP
    }

    override fun update() {
        if(_currentState == Lift.LiftStates.CLAMP_CENTER)
            setDifPos(clamp(_yPos + _deltaTime.seconds() * _yVelocity, -90.0, 90.0), clamp(_xPos + _deltaTime.seconds() * _xVelocity, -90.0, 90.0))

        _deltaTime.reset()
    }

    override fun start() {
        _deltaTime.reset()
        setDifPos(0.0, 0.0)
        clamp = ClampPosition.SERVO_CLAMP
    }
}
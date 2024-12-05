package org.firstinspires.ftc.teamcode.modules.intake

import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.modules.lift.Lift
import org.firstinspires.ftc.teamcode.modules.lift.Lift.SetLiftStateEvent
import org.firstinspires.ftc.teamcode.utils.configs.Configs
class Intake() : IRobotModule {
    class SetClampEvent(val position: ClampPosition): IEvent
    class SetDifPosEvent(val yRot: Double, val xRot: Double): IEvent

    private lateinit var _servoClamp: Servo

    private lateinit var _servoDifleft: Servo
    private lateinit var _servoDifRight: Servo

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
            if(_currentState == Lift.LiftStates.CLAMP_WALL)
                setDifPos(it.yRot, 0.0)

            if(_currentState == Lift.LiftStates.CLAMP_FIELD)
                setDifPos(0.0, it.xRot)

            if(_currentState == Lift.LiftStates.CLAMP_CENTER)
                setDifPos(0.0, it.xRot)

            if(_currentState == Lift.LiftStates.MIDDLE_BASKET)
                setDifPos(it.yRot, 0.0)

            if(_currentState == Lift.LiftStates.DOWN_BASKET)
                setDifPos(it.yRot, 0.0)

            if(_currentState == Lift.LiftStates.DOWN_LAYER)
                setDifPos(0.0, it.xRot )

            if(_currentState == Lift.LiftStates.UP_LAYER)
                setDifPos(0.0, it.xRot)

            if(_currentState == Lift.LiftStates.UP_BASKET)
                setDifPos(it.yRot, 0.0)
        }

        bus.subscribe(SetLiftStateEvent::class){
            _currentState = it.state

            //тут проверки

            if(_currentState == Lift.LiftStates.SETUP)
                setDifPos(90.0, 90.0)

            if(_currentState == Lift.LiftStates.UP_BASKET)
                setDifPos(90.0, 90.0)

            if(_currentState == Lift.LiftStates.UP_LAYER)
                setDifPos(90.0, 90.0)

            if(_currentState == Lift.LiftStates.MIDDLE_BASKET)
                setDifPos(90.0, 90.0)

            if(_currentState == Lift.LiftStates.CLAMP_WALL)
                setDifPos(90.0, 90.0)

            if(_currentState == Lift.LiftStates.CLAMP_FIELD)
                setDifPos(90.0, 90.0)

            if(_currentState == Lift.LiftStates.CLAMP_CENTER)
                setDifPos(90.0, 90.0)

            if(_currentState == Lift.LiftStates.DOWN_LAYER)
                setDifPos(90.0, 90.0)

            if(_currentState == Lift.LiftStates.DOWN_LAYER)
                setDifPos(90.0, 90.0)

        }
    }

    class SetClampPoseEvent(var pose: ClampPosition): IEvent

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
        _servoDifRight.position = (yRot + xRot)/Configs.IntakeConfig.MAX
        _servoDifleft.position = (xRot - yRot)/Configs.IntakeConfig.MAX
    }

    enum class ClampPosition// захват
    {
        SERVO_CLAMP,
        SERVO_UNCLAMP
    }
}
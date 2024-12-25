package org.firstinspires.ftc.teamcode.modules.mainControl.actions

import com.acmerobotics.roadrunner.Trajectory
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.modules.intake.Lift
import org.firstinspires.ftc.teamcode.modules.mainControl.runner.RRTrajectorySegment
import org.firstinspires.ftc.teamcode.modules.mainControl.runner.TrajectorySegmentRunner
import org.firstinspires.ftc.teamcode.modules.mainControl.runner.TurnSegment
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Orientation

interface IAction{
    fun update()

    fun end()

    fun isEnd(): Boolean

    fun start()
}

class FollowRRTrajectory(private val _eventBus: EventBus, private val _trajectory: List<Trajectory>): IAction{
    override fun update() {}

    override fun end() {}

    override fun isEnd() = _eventBus.invoke(TrajectorySegmentRunner.RequestIsEndTrajectoryEvent()).isEnd

    override fun start() {
        _eventBus.invoke(TrajectorySegmentRunner.RunTrajectorySegmentEvent(RRTrajectorySegment(_trajectory)))
    }
}

class TurnAction(private val _eventBus: EventBus, private val _startOrientation: Orientation, private val _endAngle: Angle): IAction{
    override fun update() {}

    override fun end() {}

    override fun isEnd() = _eventBus.invoke(TrajectorySegmentRunner.RequestIsEndTrajectoryEvent()).isEnd

    override fun start() {
        _eventBus.invoke(TrajectorySegmentRunner.RunTrajectorySegmentEvent(TurnSegment((_startOrientation.angl - _endAngle).angle, _startOrientation)))
    }
}

class WaitAction(private val _secTime: Double): IAction {
    private val _timer = ElapsedTime()

    override fun update() {

    }

    override fun end() {

    }

    override fun isEnd() = _timer.seconds() > _secTime

    override fun start() {
        _timer.reset()
    }
}

class LiftAction(private val _eventBus: EventBus): IAction{
    override fun update() {

    }

    override fun end() {

    }

    override fun isEnd() = true//_eventBus.invoke(Lift.RequestLiftAtTargetEvent(false)).atTarget

    override fun start() {
       // _eventBus.invoke(Lift.SetLiftStateEvent(_liftState))
    }
}

class OpenClampAction(private val _eventBus: EventBus): IAction{
    private val _timer = ElapsedTime()

    override fun update() {

    }

    override fun end() {

    }

    override fun isEnd() = _timer.seconds() > Configs.IntakeConfig.LIFT_TIME

    override fun start() {
        //_eventBus.invoke(Intake.SetClampStateEvent(Intake.ClampPosition.SERVO_UNCLAMP))

        _timer.reset()
    }
}
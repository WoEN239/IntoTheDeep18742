package org.firstinspires.ftc.teamcode.modules.mainControl.actions

import com.acmerobotics.roadrunner.Trajectory
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.modules.intake.IntakeManager
import org.firstinspires.ftc.teamcode.modules.mainControl.runner.RRTrajectorySegment
import org.firstinspires.ftc.teamcode.modules.mainControl.runner.TrajectorySegmentRunner
import org.firstinspires.ftc.teamcode.modules.mainControl.runner.TurnSegment
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Orientation

interface IAction {
    fun update()

    fun end()

    fun isEnd(): Boolean

    fun start()
}

interface ITransportAction : IAction {
    fun getEndOrientation(): Orientation

    companion object {
        fun getEndOrientation(actions: List<IAction>): Orientation {
            for (i in actions.indices.reversed()) {
                if (actions[i] is ITransportAction)
                    return (actions[i] as ITransportAction).getEndOrientation()
            }

            throw Exception("trajectory not contains transport actions")
        }
    }
}

class FollowRRTrajectory(private val _eventBus: EventBus, trajectory: List<Trajectory>) : ITransportAction {
    private val _segment = RRTrajectorySegment(trajectory)

    override fun update() {}

    override fun end() {}

    override fun isEnd() =
        _eventBus.invoke(TrajectorySegmentRunner.RequestIsEndTrajectoryEvent()).isEnd

    override fun start() {
        _eventBus.invoke(TrajectorySegmentRunner.RunTrajectorySegmentEvent(_segment))
    }

    override fun getEndOrientation() = _segment.targetOrientation(_segment.duration())
}

class TurnAction(private val _eventBus: EventBus, startOrientation: Orientation, endAngle: Angle) : ITransportAction {
    private val _segment =
        TurnSegment((startOrientation.angl - endAngle).angle, startOrientation)

    override fun update() {}

    override fun end() {}

    override fun isEnd() =
        _eventBus.invoke(TrajectorySegmentRunner.RequestIsEndTrajectoryEvent()).isEnd

    override fun start() {
        _eventBus.invoke(TrajectorySegmentRunner.RunTrajectorySegmentEvent(_segment))
    }

    override fun getEndOrientation() = _segment.targetOrientation(_segment.duration())
}

class WaitAction(private val _secTime: Double) : IAction {
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

class LiftAction(private val _eventBus: EventBus, val pos: IntakeManager.LiftPosition, val extensionPos: Double = 0.0) : IAction {
    private val _timer = ElapsedTime()

    override fun update() {

    }

    override fun end() {

    }

    override fun isEnd() = _timer.seconds() > Configs.LiftConfig.LIFT_TIMER

    override fun start() {
        _eventBus.invoke(IntakeManager.EventSetLiftPose(pos))
        _eventBus.invoke(IntakeManager.EventSetExtensionPosition(extensionPos))
    }
}

class ClampAction(private val _eventBus: EventBus, val pos: Intake.ClampPosition) : IAction {
    private val _timer = ElapsedTime()

    override fun update() {

    }

    override fun end() {

    }

    override fun isEnd() = _timer.seconds() > Configs.IntakeConfig.CLAMP_TIME

    override fun start() {
        _eventBus.invoke(IntakeManager.EventSetClampPose(pos))

        _timer.reset()
    }
}
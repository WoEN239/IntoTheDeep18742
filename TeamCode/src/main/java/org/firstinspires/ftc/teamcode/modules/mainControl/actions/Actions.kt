package org.firstinspires.ftc.teamcode.modules.mainControl.actions

import com.acmerobotics.roadrunner.Trajectory
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.modules.intake.IntakeManager
import org.firstinspires.ftc.teamcode.modules.mainControl.runner.RRTrajectorySegment
import org.firstinspires.ftc.teamcode.modules.mainControl.runner.TrajectorySegmentRunner
import org.firstinspires.ftc.teamcode.modules.mainControl.runner.TurnSegment
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

class FollowRRTrajectory(private val _eventBus: EventBus, trajectory: List<Trajectory>) :
    ITransportAction {
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

class TurnAction(private val _eventBus: EventBus, startOrientation: Orientation, endAngle: Angle) :
    ITransportAction {
    private val _segment =
        TurnSegment((endAngle - startOrientation.angl).angle, startOrientation)

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

class LiftAction(
    private val _eventBus: EventBus,
    val pos: IntakeManager.LiftPosition,
    val extensionPos: Double = 0.0
) : IAction {
    override fun update() {

    }

    override fun end() {

    }

    override fun isEnd() = _eventBus.invoke(IntakeManager.RequestLiftAtTargetEvent()).target!!

    override fun start() {
        _eventBus.invoke(IntakeManager.EventSetLiftPose(pos))
        _eventBus.invoke(IntakeManager.EventSetExtensionPosition(extensionPos))
    }
}

class ClampAction(private val _eventBus: EventBus, val pos: Intake.ClampPosition) : IAction {
    override fun update() {

    }

    override fun end() {

    }

    override fun isEnd() = _eventBus.invoke(IntakeManager.RequestIntakeAtTarget()).target!!

    override fun start() {
        _eventBus.invoke(IntakeManager.EventSetClampPose(pos))
    }
}

class WaitLiftAction(private val _eventBus: EventBus) : IAction {
    override fun update() {}

    override fun end() {}

    override fun isEnd() = _eventBus.invoke(IntakeManager.RequestLiftAtTargetEvent()).target!!

    override fun start() {}
}

class ParallelActions(
    private val _actions: Array<ArrayList<IAction>>,
    private val _exitType: ExitType
) : ITransportAction {
    enum class ExitType {
        AND, OR
    }

    override fun getEndOrientation(): Orientation {
        for(i in _actions){
            for (j in i.indices.reversed()) {
                if(i[j] is ITransportAction)
                    return (i[j] as ITransportAction).getEndOrientation()
            }
        }

        throw Exception("trajectory not contains transport actions")
    }

    override fun update() {
        for (i in _actions) {
            if (!i.isEmpty()) {
                i[0].update()

                if (i[0].isEnd()) {
                    i[0].end()

                    i.removeAt(0)

                    if(!i.isEmpty())
                        i[0].start()
                }
            }
        }
    }

    override fun end() {

    }

    override fun isEnd(): Boolean {
        for (i in _actions)
            if(!i.isEmpty() && i[0].isEnd()){
                if(_exitType == ExitType.OR && i.isEmpty())
                    return true
                else if(_exitType == ExitType.AND && !i.isEmpty())
                    return false
            }

        return true
    }

    override fun start() {
        for(i in _actions)
            if(!i.isEmpty())
                i[0].start()
    }
}

class DifAction(val eventBus: EventBus, val dir: DifDirection): IAction{
    enum class DifDirection{
        NEXT,
        PREVIOUS
    }

    override fun update() {

    }

    override fun end() {
    }

    override fun isEnd() = true

    override fun start() {
        if(dir == DifDirection.NEXT)
            eventBus.invoke(IntakeManager.NextDifPos())
        else
            eventBus.invoke(IntakeManager.PreviousDifPos())
    }

}
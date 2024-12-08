package org.firstinspires.ftc.teamcode.modules.mainControl.actions

import com.acmerobotics.roadrunner.Vector2d
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.modules.driveTrain.DriveTrain
import org.firstinspires.ftc.teamcode.modules.mainControl.runner.TrajectorySegmentRunner
import org.firstinspires.ftc.teamcode.utils.timer.Timer
import org.firstinspires.ftc.teamcode.utils.timer.Timers
import org.firstinspires.ftc.teamcode.utils.units.Orientation
import org.firstinspires.ftc.teamcode.utils.units.Vec2

class ActionsRunner: IRobotModule {
    class RunActionsEvent(val actions: List<IAction>): IEvent
    class RequestIsEndActionsRun(var isEnd: Boolean): IEvent

    private val _actions = ArrayList<IAction>()

    private lateinit var _eventBus: EventBus

    override fun init(collector: BaseCollector, bus: EventBus) {
        _eventBus = bus

        bus.subscribe(RunActionsEvent::class){
            if(_actions.isEmpty())
                it.actions[0].start()

            _actions.addAll(it.actions)
        }

        bus.subscribe(RequestIsEndActionsRun::class){
            it.isEnd = _actions.isEmpty()
        }

        val actions = arrayListOf<IAction>()

        actions.add(FollowRRTrajectory(bus, TrajectorySegmentRunner.newRRTrajectory(
            Orientation(collector.gameSettings.startPosition.position, collector.gameSettings.startPosition.angle))
            .strafeTo(Vector2d(0.0, 100.0))
            .strafeToLinearHeading(Vector2d(140.0, 70.0), Math.toRadians(45.0))
            .build()))

        _eventBus.invoke(RunActionsEvent(actions))

    }

    override fun update() {
        if(_actions.isEmpty())
            return

        _actions[0].update()

        if(_actions[0].isEnd()) {
            _actions[0].end()
            _actions.removeAt(0)

            if(!_actions.isEmpty())
                _actions[0].start()
        }
    }

    override fun start() {
        /*_eventBus.invoke(DriveTrain.SetDrivePowerEvent(Vec2(0.3, 0.0), 0.0))

        Timers.newTimer().start(5.0){
            _eventBus.invoke(DriveTrain.SetDrivePowerEvent(Vec2(0.0, 0.0), 0.0))
        }*/
    }
}
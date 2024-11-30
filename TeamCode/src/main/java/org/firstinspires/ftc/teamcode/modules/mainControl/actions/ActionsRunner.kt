package org.firstinspires.ftc.teamcode.modules.mainControl.actions

import com.acmerobotics.roadrunner.Vector2d
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.modules.mainControl.runner.TrajectorySegmentRunner
import org.firstinspires.ftc.teamcode.utils.units.Orientation

class ActionsRunner: IRobotModule {
    class RunActionsEvent(val actions: List<IAction>): IEvent
    class RequestIsEndActionsRun(var isEnd: Boolean): IEvent

    private val _actions = ArrayList<IAction>()

    override fun init(collector: BaseCollector, bus: EventBus) {
        bus.subscribe(RunActionsEvent::class){
            if(_actions.isEmpty())
                it.actions[0].start()

            _actions.addAll(it.actions)
        }

        bus.subscribe(RequestIsEndActionsRun::class){
            it.isEnd = _actions.isEmpty()
        }

        val actions = ArrayList<IAction>()

        when(collector.gameSettings.startPosition){
            BaseCollector.GameStartPosition.RED_FORWARD -> {
                actions.add(FollowRRTrajectory(bus, TrajectorySegmentRunner.newRRTrajectory(Orientation.ZERO)
                    .strafeTo(Vector2d(20.0, 20.0))

                    .build()))
            }

            BaseCollector.GameStartPosition.RED_BACK -> TODO()
            BaseCollector.GameStartPosition.BLUE_FORWARD -> TODO()
            BaseCollector.GameStartPosition.BLUE_BACK -> TODO()
            BaseCollector.GameStartPosition.NONE -> TODO()
        }

        bus.invoke(RunActionsEvent(actions))
    }

    override fun update() {
        if(_actions.isEmpty)
            return

        _actions[0].update()

        if(_actions[0].isEnd()) {
            _actions[0].end()
            _actions.removeAt(0)

            if(!_actions.isEmpty())
                _actions[0].start()
        }
    }
}
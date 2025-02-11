package org.firstinspires.ftc.teamcode.modules.mainControl.actions

import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.trajectoryes.BaskedTrajectory
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.trajectoryes.HumanTrajectory
import org.firstinspires.ftc.teamcode.utils.units.Orientation

class ActionsRunner: IRobotModule {
    class RunActionsEvent(val actions: List<IAction>): IEvent
    class RequestIsEndActionsRun(var isEnd: Boolean): IEvent

    private val _actions = ArrayList<IAction>()

    private lateinit var _eventBus: EventBus

    override fun init(collector: BaseCollector, bus: EventBus) {
        _eventBus = bus

        bus.subscribe(RunActionsEvent::class){
            _actions.addAll(it.actions)
        }

        bus.subscribe(RequestIsEndActionsRun::class){
            it.isEnd = _actions.isEmpty()
        }

        when(collector.parameters.oldStartPosition){
            BaseCollector.GameStartPosition.RED_HUMAN -> HumanTrajectory()
            BaseCollector.GameStartPosition.RED_BASKET -> BaskedTrajectory()
            BaseCollector.GameStartPosition.BLUE_HUMAN -> HumanTrajectory()
            BaseCollector.GameStartPosition.BLUE_BASKET -> BaskedTrajectory()
            BaseCollector.GameStartPosition.NONE -> throw Exception("none is not start auto pos")
        }.runTrajectory(bus, Orientation(collector.parameters.oldStartPosition.position, collector.parameters.oldStartPosition.angle))
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
        if(!_actions.isEmpty())
            _actions[0].start()
    }
}
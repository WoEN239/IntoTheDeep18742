package org.firstinspires.ftc.teamcode.modules.mainControl.actions

import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.modules.driveTrain.DriveTrain
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.modules.intake.IntakeManager
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.trajectoryes.BlueBaskedTrajectory
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.trajectoryes.BlueHumanTrajectory
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.trajectoryes.RedBaskedTrajectory
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.trajectoryes.RedHumanTrajectory
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

//        val actions = arrayListOf<IAction>()
//
//        actions.add(FollowRRTrajectory(bus, TrajectorySegmentRunner.newRRTrajectory(Orientation(collector.gameSettings.startPosition.position, collector.gameSettings.startPosition.angle))
//            .strafeTo(Vector2d(55.0, -10.0))
//            .build()))
//
//        actions.add(WaitAction(1.0))
//
//        actions.add(TurnAction(bus, getEndOrientation(actions), Angle.ofDeg(180.0)))
//
//        actions.add(FollowRRTrajectory(bus, TrajectorySegmentRunner.newRRTrajectory(getEndOrientation(actions))
//                .strafeTo(Vector2d(60.0, 43.0))
//                .strafeTo(Vector2d(132.0, 52.0))
//
//                .strafeTo(Vector2d(125.0, 65.0))
//                .strafeTo(Vector2d(60.0, 70.0))
//                .strafeTo(Vector2d(110.0, 62.0))
//
//                .strafeTo(Vector2d(120.0, 80.0))
//                .strafeTo(Vector2d(60.0, 90.0))
//                .strafeTo(Vector2d(121.0, 90.0))
//
//                .strafeTo(Vector2d(127.0, 120.0))
//                .strafeTo(Vector2d(60.0, 120.0))
//                .strafeTo(Vector2d(20.0, 100.0))
//            .build()))
//
//        _eventBus.invoke(RunActionsEvent(actions))

        when(collector.parameters.oldStartPosition){
            BaseCollector.GameStartPosition.RED_HUMAN -> RedHumanTrajectory()
            BaseCollector.GameStartPosition.RED_BASKET -> RedBaskedTrajectory()
            BaseCollector.GameStartPosition.BLUE_HUMAN -> BlueHumanTrajectory()
            BaseCollector.GameStartPosition.BLUE_BASKET -> BlueBaskedTrajectory()
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
        /*_eventBus.invoke(DriveTrain.SetDrivePowerEvent(Vec2(-0.3, 0.0), 0.0))

        Timers.newTimer().start(0.7){
            _eventBus.invoke(DriveTrain.SetDrivePowerEvent(Vec2(0.0, 0.0), 0.0))
        }*/
    }
}
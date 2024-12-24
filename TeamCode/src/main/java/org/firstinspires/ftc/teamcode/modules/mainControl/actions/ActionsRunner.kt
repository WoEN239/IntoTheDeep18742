package org.firstinspires.ftc.teamcode.modules.mainControl.actions

import com.acmerobotics.roadrunner.Vector2d
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.modules.driveTrain.DriveTrain
import org.firstinspires.ftc.teamcode.modules.lift.Lift
import org.firstinspires.ftc.teamcode.modules.mainControl.runner.TrajectorySegmentRunner
import org.firstinspires.ftc.teamcode.utils.timer.Timer
import org.firstinspires.ftc.teamcode.utils.timer.Timers
import org.firstinspires.ftc.teamcode.utils.units.Angle
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
            .strafeTo(Vector2d(35.0, -70.0))
            .build()))

        actions.add(TurnAction(bus, Orientation(Vec2(20.0, -55.0), collector.gameSettings.startPosition.angle),
            Angle.ofDeg(45.0)))

        actions.add(LiftAction(bus, Lift.LiftStates.UP_BASKET))

        actions.add(FollowRRTrajectory(bus, TrajectorySegmentRunner.newRRTrajectory(
            Orientation(Vec2(20.0, -55.0), Angle.ofDeg(45.0)))
            .strafeTo(Vector2d(30.0, -65.0))
            .build()))

        actions.add(OpenClampAction(_eventBus))

        actions.add(TurnAction(_eventBus, Orientation(Vec2(30.0, -65.0), Angle.ofDeg(45.0)), Angle.ofDeg(-90.0)))

        actions.add(FollowRRTrajectory(bus, TrajectorySegmentRunner.newRRTrajectory(
            Orientation(Vec2(30.0, -65.0), Angle.ofDeg(-90.0)))
            .strafeTo(Vector2d(20.0, 150.0))
            .build()))

        _eventBus.invoke(RunActionsEvent(actions))

//        val actions = arrayListOf<IAction>()
//
//        actions.add(FollowRRTrajectory(bus, TrajectorySegmentRunner.newRRTrajectory(
//            Orientation(collector.gameSettings.startPosition.position, collector.gameSettings.startPosition.angle))
//            .strafeTo(Vector2d(55.0, -10.0))
//            .build()))
//
//        actions.add(WaitAction(1.0))
//
//        actions.add(TurnAction(bus, Orientation(Vec2(55.0, -40.0), collector.gameSettings.startPosition.angle), Angle.ofDeg(180.0)))
//
//        actions.add(FollowRRTrajectory(bus, TrajectorySegmentRunner.newRRTrajectory(
//            Orientation(Vec2(55.0, -40.0), Angle.ofDeg(180.0)))                                                                                                                   //== gay
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
package org.firstinspires.ftc.teamcode.modules.mainControl.actions

import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.modules.mainControl.runner.TrajectoryRunner
import org.firstinspires.ftc.teamcode.modules.mainControl.runner.TrajectoryRunner.RunTrajectoryEvent

interface Action{
    fun update()

    fun end()

    fun isEnd(): Boolean

    fun start()
}

class FollowTrajectory(private val _eventBus: EventBus, val builder: TrajectoryRunner.TrajectoryActionBuilder): Action{
    override fun update() {

    }

    override fun end() {

    }

    override fun isEnd(): Boolean {
        val end = TrajectoryRunner.RequestIsEndTrajectoryEvent()

        return end.isEnd
    }

    override fun start() {
        _eventBus.invoke(RunTrajectoryEvent(builder))
    }
}
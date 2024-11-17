package org.firstinspires.ftc.teamcode.modules.mainControl.runner

import com.acmerobotics.roadrunner.TrajectoryBuilder
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.modules.mainControl.runner.ActionRunner.NewActionsEvent
import org.firstinspires.ftc.teamcode.modules.mainControl.runner.ActionRunner.NewRRBuilder
import org.firstinspires.ftc.teamcode.modules.mainControl.runner.ActionRunner.RunActionsEvent

object ActionRunnerHelper: IRobotModule {
    lateinit var _eventBus: EventBus

    override fun init(collector: BaseCollector, bus: EventBus) {
        _eventBus = bus
    }

    fun newAB(): ActionRunner.ActionsBuilder{
        val builder = NewActionsEvent(null)

        _eventBus.invoke(builder)

        return builder.builder!!
    }

    fun newTB(): TrajectoryBuilder{
        var builder = NewRRBuilder(null)

        _eventBus.invoke(builder)

        return builder.builder!!
    }

    fun runActions(action: ActionRunner.ActionsBuilder){
        _eventBus.invoke(RunActionsEvent(action))
    }
}
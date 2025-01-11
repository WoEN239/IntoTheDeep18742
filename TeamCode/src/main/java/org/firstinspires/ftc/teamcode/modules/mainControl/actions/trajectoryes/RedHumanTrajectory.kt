package org.firstinspires.ftc.teamcode.modules.mainControl.actions.trajectoryes

import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.clamp
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.modules.intake.IntakeManager
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.ActionsRunner
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.ClampAction
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.FollowRRTrajectory
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.IAction
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.ITransportAction.Companion.getEndOrientation
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.LiftAction
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.WaitAction
import org.firstinspires.ftc.teamcode.modules.mainControl.runner.TrajectorySegmentRunner.Companion.newRRTrajectory
import org.firstinspires.ftc.teamcode.utils.units.Orientation

class RedHumanTrajectory: ITrajectoryBuilder {
    override fun runTrajectory(eventBus: EventBus, startOrientation: Orientation){
        val actions = arrayListOf<IAction>()

        extracted(actions, eventBus, startOrientation)


        extracted(actions, eventBus, startOrientation)

        actions.add(WaitAction(5.0))

        actions.add(LiftAction(eventBus, IntakeManager.LiftPosition.UP_BASKED))

        actions.add(FollowRRTrajectory(eventBus, newRRTrajectory(getEndOrientation(actions))
            .strafeTo(Vector2d(20.0, -85.0))
            .build())
            )
        actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_UNCLAMP))
        actions.add(FollowRRTrajectory(eventBus, newRRTrajectory(getEndOrientation(actions))
            .strafeTo(Vector2d(150.0,-159.0))
            .build())
        )

        eventBus.invoke(ActionsRunner.RunActionsEvent(actions))
    }
    private fun extracted(
        actions: ArrayList<IAction>,
        eventBus: EventBus,
        startOrientation: Orientation
    ) {
        actions.add(
            FollowRRTrajectory(
                eventBus, newRRTrajectory(startOrientation)
                    .build()
            )
        )
    }
}
package org.firstinspires.ftc.teamcode.modules.mainControl.actions.trajectoryes

import com.acmerobotics.roadrunner.Vector2d
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.modules.intake.IntakeManager
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.ActionsRunner.RunActionsEvent
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.ClampAction
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.FollowRRTrajectory
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.IAction
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.ITransportAction.Companion.getEndOrientation
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.LiftAction
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.WaitAction
import org.firstinspires.ftc.teamcode.modules.mainControl.runner.TrajectorySegmentRunner
import org.firstinspires.ftc.teamcode.utils.units.Orientation
import java.lang.Math.toRadians

class BlueHumanTrajectory: ITrajectoryBuilder {
    override fun runTrajectory(eventBus: EventBus, startOrientation: Orientation){
        val actions = arrayListOf<IAction>()

        fun layer(orientation: Orientation) {
            actions.add(
                FollowRRTrajectory(
                    eventBus, TrajectorySegmentRunner.newRRTrajectory(orientation)
                        .strafeToLinearHeading(Vector2d(0.0, 86.0), toRadians(90.0))
                        .build()
                )
            )

            actions.add(LiftAction(eventBus, IntakeManager.LiftPosition.UP_LAYER))
            actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_UNCLAMP))
            actions.add(WaitAction(0.9))
        }

        layer(startOrientation)

        actions.add(FollowRRTrajectory(eventBus, TrajectorySegmentRunner.newRRTrajectory(getEndOrientation(actions))
            .strafeTo(Vector2d(-80.0, 112.0))
            .strafeTo(Vector2d(-80.0, 40.0))
            .strafeTo(Vector2d(-112.0, 40.0))
            .strafeTo(Vector2d(-112.0, 122.0))
            .strafeTo(Vector2d(-112.0, 40.0))
            .strafeTo(Vector2d(-131.0, 20.0))
            .strafeTo(Vector2d(-131.0, 122.0))
            .strafeToLinearHeading(Vector2d(-60.0, 153.0), toRadians(180.0))
            .build()))

        actions.add(LiftAction(eventBus, IntakeManager.LiftPosition.CLAMP_CENTER, 1000.0))
        actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_CLAMP))

        layer(getEndOrientation(actions))

        eventBus.invoke(RunActionsEvent(actions))
    }
}
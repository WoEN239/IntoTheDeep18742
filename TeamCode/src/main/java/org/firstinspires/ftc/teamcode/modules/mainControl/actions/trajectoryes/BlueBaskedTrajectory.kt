package org.firstinspires.ftc.teamcode.modules.mainControl.actions.trajectoryes

import com.acmerobotics.roadrunner.Vector2d
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
import java.lang.Math.toRadians

class BlueBaskedTrajectory: ITrajectoryBuilder {
    override fun runTrajectory(eventBus: EventBus, startOrientation: Orientation){
        val actions = arrayListOf<IAction>()

        actions.add(FollowRRTrajectory(eventBus, newRRTrajectory(startOrientation)
            .strafeToLinearHeading(Vector2d(5.0, 80.0), Math.toRadians(90.0))
            .build()))

        actions.add(LiftAction(eventBus, IntakeManager.LiftPosition.UP_LAYER))

        actions.add(WaitAction(2.0))

        actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_UNCLAMP))

        actions.add(WaitAction(2.0))

        actions.add(FollowRRTrajectory(eventBus, newRRTrajectory(getEndOrientation(actions))
            .strafeToLinearHeading(Vector2d(144.0, 125.0), toRadians(-90.0))
            .build()))

        actions.add(WaitAction(1.0))

        actions.add(LiftAction(eventBus, IntakeManager.LiftPosition.CLAMP_CENTER, 1000.0))

        actions.add(WaitAction(2.0))

        eventBus.invoke(ActionsRunner.RunActionsEvent(actions))
    }
}
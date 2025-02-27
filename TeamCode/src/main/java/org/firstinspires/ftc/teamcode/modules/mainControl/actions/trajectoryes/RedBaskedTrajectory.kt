package org.firstinspires.ftc.teamcode.modules.mainControl.actions.trajectoryes

import com.acmerobotics.roadrunner.Vector2d
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.modules.intake.IntakeManager
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.ActionsRunner
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.ClampAction
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.DifAction
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.FollowRRTrajectory
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.IAction
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.ITransportAction.Companion.getEndOrientation
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.LiftAction
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.WaitAction
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.WaitLiftAction
import org.firstinspires.ftc.teamcode.modules.mainControl.runner.TrajectorySegmentRunner.Companion.newRRTrajectory
import org.firstinspires.ftc.teamcode.utils.units.Orientation
import java.lang.Math.toRadians

class RedBaskedTrajectory: ITrajectoryBuilder {
    override fun runTrajectory(eventBus: EventBus, startOrientation: Orientation){
        val actions = arrayListOf<IAction>()

        fun runToBasket(startOrientation: Orientation) {
            actions.add(
                FollowRRTrajectory(
                    eventBus, newRRTrajectory(startOrientation)
                        .strafeToLinearHeading(Vector2d(-129.5, -121.5), toRadians(90.0 - 45.0))
                        .build()
                )
            )
        }

        fun basket() {
            actions.add(LiftAction(eventBus, IntakeManager.LiftPosition.UP_BASKED))

            actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_UNCLAMP))

            actions.add(WaitLiftAction(eventBus))
        }

        fun clampStick(extension: Double, isDif: Boolean = false){
            actions.add(WaitLiftAction(eventBus))

            actions.add(WaitAction(0.1))

            actions.add(LiftAction(eventBus, IntakeManager.LiftPosition.CLAMP_CENTER, extension))

            actions.add(WaitLiftAction(eventBus))

            if(isDif)
                for(i in 0..2)
                    actions.add(DifAction(eventBus, DifAction.DifDirection.NEXT))

            actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_CLAMP))

            actions.add(WaitLiftAction(eventBus))
        }

        runToBasket(startOrientation)
        basket()

        actions.add(
            FollowRRTrajectory(
                eventBus, newRRTrajectory(getEndOrientation(actions))
                    .strafeToLinearHeading(Vector2d(-119.9, -126.7), toRadians(90.0))
                    .build()
            )
        )

        clampStick(850.0)
        runToBasket(getEndOrientation(actions))

        basket()

        actions.add(
            FollowRRTrajectory(
                eventBus, newRRTrajectory(getEndOrientation(actions))
                    .strafeToLinearHeading(Vector2d(-138.7, -118.6), toRadians(90.0))
                    .build()
            )
        )

        clampStick(750.0)
        runToBasket(getEndOrientation(actions))
        basket()

        actions.add(
            FollowRRTrajectory(
                eventBus, newRRTrajectory(getEndOrientation(actions))
                    .strafeToLinearHeading(Vector2d(-139.0, -118.6), toRadians(90.0 - 40))
                    .build()
            )
        )

        clampStick(750.0, true)
        runToBasket(getEndOrientation(actions))
        basket()

        actions.add(
            FollowRRTrajectory(
                eventBus, newRRTrajectory(getEndOrientation(actions))
                    .strafeToLinearHeading(Vector2d(-31.0, 0.0), toRadians(180.0))
                    .build()
            )
        )

        actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_CLAMP))
        actions.add(LiftAction(eventBus, IntakeManager.LiftPosition.UP_LAYER))

        eventBus.invoke(ActionsRunner.RunActionsEvent(actions))
    }
}
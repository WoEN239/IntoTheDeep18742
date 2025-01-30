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
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.TurnAction
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.WaitAction
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.WaitLiftAction
import org.firstinspires.ftc.teamcode.modules.mainControl.runner.TrajectorySegmentRunner.Companion.newRRTrajectory
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Orientation
import org.firstinspires.ftc.teamcode.utils.units.Vec2
import java.lang.Math.toRadians

class BlueBaskedTrajectory : ITrajectoryBuilder {
    override fun    runTrajectory(eventBus: EventBus, startOrientation: Orientation) {
        val actions = arrayListOf<IAction>()

        fun runToBasket(startOrientation: Orientation) {
            actions.add(
                FollowRRTrajectory(
                    eventBus, newRRTrajectory(startOrientation)
                        .strafeToLinearHeading(Vector2d(135.4, 127.4), toRadians(-90.0 - 45.0))
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

            actions.add(WaitAction(0.1))

            actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_CLAMP))

            actions.add(WaitLiftAction(eventBus))
        }

        runToBasket(startOrientation)
        basket()

        actions.add(
            FollowRRTrajectory(
                eventBus, newRRTrajectory(getEndOrientation(actions))
                    .strafeToLinearHeading(Vector2d(119.6, 126.9), toRadians(-90.0))
                    .build()
            )
        )

        clampStick(950.0)
        runToBasket(getEndOrientation(actions))

        basket()

        actions.add(
            FollowRRTrajectory(
                eventBus, newRRTrajectory(getEndOrientation(actions))
                    .strafeToLinearHeading(Vector2d(142.7, 119.6), toRadians(-90.0))
                    .build()
            )
        )

        clampStick(750.0)
        runToBasket(getEndOrientation(actions))
        basket()

        actions.add(
            FollowRRTrajectory(
                eventBus, newRRTrajectory(getEndOrientation(actions))
                    .strafeToLinearHeading(Vector2d(139.0, 108.6), toRadians(-90.0 + 40))
                    .build()
            )
        )

        clampStick(750.0, true)
        runToBasket(getEndOrientation(actions))
        basket()

        actions.add(
            FollowRRTrajectory(
                eventBus, newRRTrajectory(getEndOrientation(actions))
                    .strafeToLinearHeading(Vector2d(-93.0, 135.0), toRadians(0.0))
                    .build()
            )
        )

        eventBus.invoke(ActionsRunner.RunActionsEvent(actions))
    }
}
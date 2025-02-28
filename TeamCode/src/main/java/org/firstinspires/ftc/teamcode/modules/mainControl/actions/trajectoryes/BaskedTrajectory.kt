package org.firstinspires.ftc.teamcode.modules.mainControl.actions.trajectoryes

import com.acmerobotics.roadrunner.Pose2d
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
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.ParallelActions
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.WaitAction
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.WaitLiftAction
import org.firstinspires.ftc.teamcode.modules.mainControl.runner.TrajectorySegmentRunner.Companion.newRRTrajectory
import org.firstinspires.ftc.teamcode.utils.units.Orientation
import java.lang.Math.toRadians

class BaskedTrajectory : ITrajectoryBuilder {
    override fun runTrajectory(eventBus: EventBus, startOrientation: Orientation) {
        val actions = arrayListOf<IAction>()

        fun runToBasket(startOrientation: Orientation): ArrayList<IAction> {
            val acts = arrayListOf<IAction>()

            acts.add(
                ParallelActions(
                    arrayOf(
                        arrayListOf(
                            FollowRRTrajectory(
                                eventBus, newRRTrajectory(startOrientation)
                                    .strafeToLinearHeading(
                                        Vector2d(135.0, 132.3),
                                        toRadians(-90.0 - 45.0)
                                    )
                                    .build()
                            )
                        ),
                        arrayListOf(
                            WaitLiftAction(eventBus),
                            WaitAction(0.1),
                            LiftAction(eventBus, IntakeManager.LiftPosition.UP_BASKED)
                        )
                    ),
                    ParallelActions.ExitType.AND
                )
            )

            return acts
        }

        fun basket(extension: Double = -1.0): ArrayList<IAction> {
            val acts = arrayListOf<IAction>()

            acts.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_UNCLAMP))

            acts.add(WaitLiftAction(eventBus))

            if (extension > 0.0)
                acts.add(
                    LiftAction(
                        eventBus,
                        IntakeManager.LiftPosition.CLAMP_CENTER,
                        extension
                    )
                )

            return acts
        }


        fun clampStick(isDif: Boolean = false): ArrayList<IAction> {
            val acts = arrayListOf<IAction>()

            acts.add(WaitLiftAction(eventBus))

            if (isDif)
                for (i in 0..2)
                    acts.add(DifAction(eventBus, DifAction.DifDirection.NEXT))

            acts.add(WaitAction(0.2))

            acts.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_CLAMP))

            acts.add(WaitLiftAction(eventBus))

            return acts
        }

        actions.addAll(runToBasket(startOrientation))

        actions.add(
            ParallelActions(
                arrayOf(
                    basket(950.0), arrayListOf(
                        WaitAction(0.1),
                        FollowRRTrajectory(
                            eventBus, newRRTrajectory(getEndOrientation(actions))
                                .strafeToLinearHeading(Vector2d(119.6, 126.9), toRadians(-90.0))
                                .build()
                        )
                    )
                ), ParallelActions.ExitType.AND
            )
        )

        actions.addAll(clampStick())
        actions.addAll(runToBasket(getEndOrientation(actions)))

        actions.add(
            ParallelActions(
                arrayOf(
                    arrayListOf(
                        WaitAction(0.1),
                        FollowRRTrajectory(
                            eventBus, newRRTrajectory(getEndOrientation(actions))
                                .strafeToLinearHeading(Vector2d(142.1, 119.6), toRadians(-90.0))
                                .build()
                        )
                    ), basket(650.0)
                ), ParallelActions.ExitType.AND
            )
        )

        actions.addAll(clampStick())
        actions.addAll(runToBasket(getEndOrientation(actions)))

        actions.add(
            ParallelActions(
                arrayOf(
                    arrayListOf(
                        WaitAction(0.1),
                        FollowRRTrajectory(
                            eventBus, newRRTrajectory(getEndOrientation(actions))
                                .strafeToLinearHeading(
                                    Vector2d(137.7, 106.7),
                                    toRadians(-90.0 + 39.5)
                                )
                                .build()
                        )
                    ), basket(780.0)
                ), ParallelActions.ExitType.AND
            )
        )

        actions.addAll(clampStick(true))
        actions.addAll(runToBasket(getEndOrientation(actions)))

        actions.add(
            ParallelActions(
                arrayOf(
                    basket(1000.0), arrayListOf(
                        WaitAction(0.1), FollowRRTrajectory(
                            eventBus,
                            newRRTrajectory(getEndOrientation(actions)).strafeToLinearHeading(
                                Vector2d(120.0, 132.3), toRadians(180.0)
                            ).build()
                        )
                    )
                ), ParallelActions.ExitType.AND
            )
        )

        actions.addAll(clampStick())
        actions.addAll(runToBasket(getEndOrientation(actions)))
        actions.addAll(basket())
//
//        actions.add(
//            FollowRRTrajectory(
//                eventBus, newRRTrajectory(getEndOrientation(actions))
//                    .setTangent(toRadians(180.0))
//                    .splineToLinearHeading(Pose2d(62.0, 0.0, toRadians(0.0)), toRadians(180.0))
//                    .build()
//            )
//        )
//
//        actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_CLAMP))
//        actions.add(LiftAction(eventBus, IntakeManager.LiftPosition.UP_LAYER))

        eventBus.invoke(ActionsRunner.RunActionsEvent(actions))
    }
}
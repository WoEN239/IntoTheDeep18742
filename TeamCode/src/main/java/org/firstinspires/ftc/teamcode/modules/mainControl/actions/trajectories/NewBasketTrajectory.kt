package org.firstinspires.ftc.teamcode.modules.mainControl.actions.trajectories

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.modules.intake.IntakeManager
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.ActionsRunner
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.AutoClampAction
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.ClampAction
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.DifAction
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.FollowRRTrajectory
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.IAction
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.ITransportAction.Companion.getEndOrientation
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.LiftAction
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.ParallelActions
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.WaitAction
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.WaitIntakeAction
import org.firstinspires.ftc.teamcode.modules.mainControl.runner.TrajectorySegmentRunner.Companion.newRRTrajectory
import org.firstinspires.ftc.teamcode.utils.units.Orientation
import java.lang.Math.toRadians

class NewBasketTrajectory : ITrajectoryBuilder {
    override fun runTrajectory(
        eventBus: EventBus,
        startOrientation: Orientation,
        teammate: BaseCollector.TeammateSate
    ) {
        val actions = arrayListOf<IAction>()

        val basketDelay = 0.1

        fun runToBasket(): ArrayList<IAction> {
            val acts = arrayListOf<IAction>()

            acts.add(
                ParallelActions(
                    arrayOf(
                        arrayListOf(
                            FollowRRTrajectory(
                                eventBus, newRRTrajectory(startOrientation)
                                    .strafeToLinearHeading(
                                        Vector2d(129.2, 133.6),
                                        toRadians(-90.0 - 45.0)
                                    )
                                    .build()
                            )
                        ),
                        arrayListOf(
                            WaitIntakeAction(eventBus),
                            LiftAction(eventBus, IntakeManager.LiftPosition.UP_BASKED)
                        )
                    ),
                    ParallelActions.ExitType.AND
                )
            )

            return acts
        }

        fun basket(extensionPos: Double = -1.0, rotateDif: Boolean = false): ArrayList<IAction> {
            val acts = arrayListOf<IAction>()

            acts.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_UNCLAMP))

            if (extensionPos > 0.0)
                acts.add(
                    LiftAction(
                        eventBus,
                        IntakeManager.LiftPosition.CLAMP_CENTER,
                        extensionPos
                    )
                )

            if (rotateDif)
                acts.add(DifAction(eventBus, 40.0))

            return acts
        }

        actions.addAll(runToBasket())

        actions.add(
            ParallelActions(
                arrayOf(
                    basket(990.0), arrayListOf(
                        WaitAction(basketDelay), FollowRRTrajectory(
                            eventBus, newRRTrajectory(getEndOrientation(actions))
                                .strafeToLinearHeading(Vector2d(117.6, 126.9), toRadians(-90.0))
                                .build()
                        )
                    )
                ), ParallelActions.ExitType.AND
            )
        )

        actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_CLAMP, false))
        actions.addAll(runToBasket())

        actions.add(
            ParallelActions(
                arrayOf(
                    basket(770.0), arrayListOf(
                        WaitAction(basketDelay),
                        FollowRRTrajectory(
                            eventBus, newRRTrajectory(getEndOrientation(actions))
                                .strafeToLinearHeading(Vector2d(141.2, 119.6), toRadians(-90.0))
                                .build()
                        )
                    )
                ), ParallelActions.ExitType.AND
            )
        )

        actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_CLAMP, false))
        actions.addAll(runToBasket())

        actions.add(
            ParallelActions(
                arrayOf(
                    basket(780.0, true), arrayListOf(
                        WaitAction(basketDelay),
                        FollowRRTrajectory(
                            eventBus, newRRTrajectory(getEndOrientation(actions))
                                .strafeToLinearHeading(
                                    Vector2d(129.7, 106.7),
                                    toRadians(-90.0 + 39.5)
                                )
                                .build()
                        )
                    )
                ), ParallelActions.ExitType.AND
            )
        )

        actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_CLAMP, false))
        actions.add(WaitAction(0.1))
        actions.addAll(runToBasket())

        if (teammate.brick) {
            actions.add(
                ParallelActions(
                    arrayOf(
                        basket(1000.0), arrayListOf(
                            WaitAction(basketDelay),
                            FollowRRTrajectory(
                                eventBus, newRRTrajectory(getEndOrientation(actions))
                                    .strafeToLinearHeading(
                                        Vector2d(112.0, 142.3), toRadians(180.0)
                                    ).build()
                            )
                        )
                    ), ParallelActions.ExitType.AND
                )
            )

            actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_CLAMP, false))
            actions.add(WaitAction(0.1))
            actions.add(
                ParallelActions(
                    arrayOf(
                        arrayListOf(
                            FollowRRTrajectory(
                                eventBus, newRRTrajectory(startOrientation)
                                    .strafeToLinearHeading(
                                        Vector2d(130.1, 122.5),
                                        toRadians(-90.0 - 45.0)
                                    )
                                    .build()
                            )
                        ),
                        arrayListOf(
                            WaitIntakeAction(eventBus),
                            LiftAction(eventBus, IntakeManager.LiftPosition.UP_BASKED)
                        )
                    ),
                    ParallelActions.ExitType.AND
                )
            )
        }

        actions.add(
            ParallelActions(
                arrayOf(
                    basket(), arrayListOf(
                        FollowRRTrajectory(
                            eventBus,
                            newRRTrajectory(getEndOrientation(actions))
                                .setTangent(toRadians(-90.0))
                                .splineToLinearHeading(
                                    Pose2d(47.0, 10.0, toRadians(180.0)),
                                    toRadians(180.0)
                                ).build()
                        )
                    )
                ), ParallelActions.ExitType.AND
            )
        )

        actions.add(AutoClampAction(eventBus))

        actions.add(
            ParallelActions(
                arrayOf(
                    arrayListOf(
                        FollowRRTrajectory(
                            eventBus, newRRTrajectory(getEndOrientation(actions))
                                .setReversed(true)
                                .splineToLinearHeading(
                                    Pose2d(91.0, 149.0, toRadians(-90.0 - 45.0)),
                                    toRadians(90.0)
                                )
                                .build()
                        )
                    ),
                    arrayListOf(
                        WaitIntakeAction(eventBus),
                        LiftAction(eventBus, IntakeManager.LiftPosition.UP_BASKED)
                    )
                ), ParallelActions.ExitType.AND
            )
        )

        actions.add(
            ParallelActions(
                arrayOf(
                    basket(), arrayListOf(
                        WaitAction(basketDelay),
                        FollowRRTrajectory(
                            eventBus,
                            newRRTrajectory(getEndOrientation(actions))
                                .setTangent(toRadians(-90.0))
                                .splineToLinearHeading(
                                    Pose2d(20.0, 30.0, toRadians(180.0)),
                                    toRadians(180.0)
                                ).build()
                        )
                    )
                ), ParallelActions.ExitType.AND
            )
        )

        actions.add(AutoClampAction(eventBus))

        actions.add(
            ParallelActions(
                arrayOf(
                    arrayListOf(
                        FollowRRTrajectory(
                            eventBus, newRRTrajectory(getEndOrientation(actions))
                                .setReversed(true)
                                .splineToLinearHeading(
                                    Pose2d(70.0, 153.0, toRadians(-90.0 - 45.0)),
                                    toRadians(90.0)
                                )
                                .build()
                        )
                    ),
                    arrayListOf(
                        WaitIntakeAction(eventBus),
                        LiftAction(eventBus, IntakeManager.LiftPosition.UP_BASKED)
                    )
                ), ParallelActions.ExitType.AND
            )
        )
        actions.addAll(basket())

        eventBus.invoke(ActionsRunner.RunActionsEvent(actions))
    }
}
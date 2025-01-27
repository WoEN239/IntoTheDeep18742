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
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.ParallelActions
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.TurnAction
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.WaitAction
import org.firstinspires.ftc.teamcode.modules.mainControl.runner.TrajectorySegmentRunner
import org.firstinspires.ftc.teamcode.modules.mainControl.runner.TrajectorySegmentRunner.Companion.newRRTrajectory
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Orientation
import java.lang.Math.toRadians

class BlueHumanTrajectory : ITrajectoryBuilder {
    override fun runTrajectory(eventBus: EventBus, startOrientation: Orientation) {
        val actions = arrayListOf<IAction>()

        actions.add(
            ParallelActions(arrayOf(
                arrayListOf(
                    FollowRRTrajectory(
                        eventBus, newRRTrajectory(startOrientation)
                            .strafeToLinearHeading(
                                Vector2d(0.0, 79.0),
                                toRadians(90.0)
                            )
                            .build()
                    )
                ),
                arrayListOf(
                    WaitAction(0.1),
                    LiftAction(eventBus, IntakeManager.LiftPosition.UP_LAYER)
                )
            ), ParallelActions.ExitType.AND)
        )

        actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_UNCLAMP))

        actions.add(
            ParallelActions(
                arrayOf(
                    arrayListOf(
                        FollowRRTrajectory(
                            eventBus, newRRTrajectory(getEndOrientation(actions))
                                .strafeToLinearHeading(
                                    Vector2d(-83.0, 117.0),
                                    toRadians(-90.0 - 45.0)
                                )
                                .build()
                        )
                    ),

                    arrayListOf(
                        WaitAction(0.1),
                        LiftAction(eventBus, IntakeManager.LiftPosition.HUMAN_ADD)
                    )

                ), ParallelActions.ExitType.AND
            )
        )

        actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_CLAMP))

        actions.add(TurnAction(eventBus, getEndOrientation(actions), Angle.ofDeg(180.0 - 45.0)))

        actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_UNCLAMP))

        actions.add(
            FollowRRTrajectory(
            eventBus, newRRTrajectory(startOrientation)
                .strafeToLinearHeading(
                    Vector2d(-78.0 - 27.0, 118.0),
                    toRadians(-90.0 - 45.0)
                )
                .build()
            )
        )

        actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_CLAMP))

        actions.add(TurnAction(eventBus, getEndOrientation(actions), Angle.ofDeg(180.0 - 45.0)))

        actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_UNCLAMP))

        actions.add(TurnAction(eventBus, getEndOrientation(actions), Angle.ofDeg(-90.0 - 45.0)))

        actions.add(
            FollowRRTrajectory(
                eventBus, newRRTrajectory(getEndOrientation(actions))
                    .strafeTo(Vector2d(-78.0 - 27.0 - 25.0, 117.0))
                    .build()
            )
        )

        actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_CLAMP))

        actions.add(
            FollowRRTrajectory(
                eventBus, newRRTrajectory(getEndOrientation(actions))
                    .strafeToLinearHeading(
                        Vector2d(-73.0, 115.0),
                        toRadians(180.0 - 45.0)
                    )
                    .build()
            )
        )

        actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_UNCLAMP))

        actions.add(ParallelActions(arrayOf(
            arrayListOf(
                FollowRRTrajectory(
                    eventBus, newRRTrajectory(getEndOrientation(actions))
                        .strafeToLinearHeading(
                            Vector2d(-93.0, 134.0),
                            toRadians(90.0)
                        )
                        .build()
                )
            ),
            arrayListOf(
                LiftAction(eventBus, IntakeManager.LiftPosition.TRANSPORT),
                LiftAction(eventBus, IntakeManager.LiftPosition.CLAMP_WALL)
            )
        ), ParallelActions.ExitType.AND))

        actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_CLAMP))

        actions.add(
            ParallelActions(
                arrayOf(
                    arrayListOf(
                        FollowRRTrajectory(
                            eventBus, newRRTrajectory(getEndOrientation(actions))
                                .splineToConstantHeading(
                                    Vector2d(2.0, 79.0),
                                    toRadians(90.0)
                                )
                                .build()
                        )
                    ),
                    arrayListOf(
                        LiftAction(eventBus, IntakeManager.LiftPosition.UP_LAYER)
                    )
                ), ParallelActions.ExitType.AND)
        )

        actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_UNCLAMP))

        eventBus.invoke(RunActionsEvent(actions))
    }
}
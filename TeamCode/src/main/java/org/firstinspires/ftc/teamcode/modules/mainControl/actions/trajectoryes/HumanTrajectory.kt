package org.firstinspires.ftc.teamcode.modules.mainControl.actions.trajectoryes

import com.acmerobotics.roadrunner.AccelConstraint
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.ProfileAccelConstraint
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
import org.firstinspires.ftc.teamcode.modules.mainControl.runner.TrajectorySegmentRunner.Companion.newRRTrajectory
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Orientation
import java.lang.Math.toRadians

class HumanTrajectory : ITrajectoryBuilder {
    override fun runTrajectory(eventBus: EventBus, startOrientation: Orientation) {
        val layerAccelConstrain = ProfileAccelConstraint(-150.0, Configs.DriveTrainConfig.MAX_TRANSLATION_ACCEL)
        val humanUnclampPos = ProfileAccelConstraint(-150.0, 150.0)

        val actions = arrayListOf<IAction>()

        actions.add(
            ParallelActions(
                arrayOf(
                    arrayListOf(
                        FollowRRTrajectory(
                            eventBus, newRRTrajectory(startOrientation)
                                .strafeToLinearHeading(
                                    Vector2d(14.2, 72.5),
                                    toRadians(90.0),
                                    accelConstraintOverride = layerAccelConstrain
                                )
                                .build()
                        )
                    ),
                    arrayListOf(
                        WaitAction(0.1),
                        LiftAction(eventBus, IntakeManager.LiftPosition.UP_LAYER)
                    )
                ), ParallelActions.ExitType.AND
            )
        )

        actions.add(WaitAction(0.1))

        actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_UNCLAMP))

        actions.add(
            ParallelActions(
                arrayOf(
                    arrayListOf(
                        FollowRRTrajectory(
                            eventBus, newRRTrajectory(getEndOrientation(actions))
                                .strafeToLinearHeading(
                                    Vector2d(-78.2, 120.2),
                                    toRadians(-90.0 - 45.0)
                                )
                                .build()
                        )
                    ),

                    arrayListOf(
                        LiftAction(eventBus, IntakeManager.LiftPosition.HUMAN_ADD)
                    )

                ), ParallelActions.ExitType.AND
            )
        )

        actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_CLAMP))

        actions.add(TurnAction(eventBus, getEndOrientation(actions), Angle.ofDeg(180.0 - 35.0)))

        actions.add(ParallelActions(arrayOf(
            arrayListOf(
                ClampAction(eventBus, Intake.ClampPosition.SERVO_UNCLAMP)
            ),
            arrayListOf(
                FollowRRTrajectory(
                    eventBus, newRRTrajectory(getEndOrientation(actions))
                        .strafeToLinearHeading(
                            Vector2d(-98.2, 118.2),
                            toRadians(-90.0 - 45.0),
                            accelConstraintOverride = humanUnclampPos
                        )
                        .build()
                )
            )
        ), ParallelActions.ExitType.AND))

        actions.add(WaitAction(0.1))

        actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_CLAMP))

        actions.add(TurnAction(eventBus, getEndOrientation(actions), Angle.ofDeg(180.0 - 35.0)))

        actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_UNCLAMP))

        actions.add(
            ParallelActions(
                arrayOf(
                    arrayListOf(
                        FollowRRTrajectory(
                            eventBus, newRRTrajectory(getEndOrientation(actions))
                                .setTangent(toRadians(-90.0))
                                .splineToLinearHeading(
                                    Pose2d(Vector2d(-95.5, 131.0),
                                    toRadians(90.0)),
                                    toRadians(180.0 - 35.0)
                                )
                                .build()
                        )
                    ),
                    arrayListOf(
                        LiftAction(eventBus, IntakeManager.LiftPosition.TRANSPORT),
                        LiftAction(eventBus, IntakeManager.LiftPosition.CLAMP_WALL)
                    )
                ), ParallelActions.ExitType.AND
            )
        )

        actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_CLAMP))

        actions.add(
            ParallelActions(
                arrayOf(
                    arrayListOf(
                        FollowRRTrajectory(
                            eventBus, newRRTrajectory(getEndOrientation(actions))
                                .setTangent(toRadians(0.0))
                                .splineToConstantHeading(Vector2d(12.0, 81.0), toRadians(-90.0), accelConstraintOverride = layerAccelConstrain)
                                .build()
                        )
                    ),
                    arrayListOf(
                        LiftAction(eventBus, IntakeManager.LiftPosition.UP_LAYER)
                    )
                ), ParallelActions.ExitType.AND
            )
        )

        actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_UNCLAMP))

        actions.add(
            ParallelActions(
                arrayOf(
                    arrayListOf(
                        FollowRRTrajectory(
                            eventBus, newRRTrajectory(getEndOrientation(actions))
                                .strafeToLinearHeading(
                                    Vector2d(-90.1, 139.6),
                                    toRadians(90.0)
                                )
                                .build()
                        )
                    ),
                    arrayListOf(
                        LiftAction(eventBus, IntakeManager.LiftPosition.TRANSPORT),
                        LiftAction(eventBus, IntakeManager.LiftPosition.CLAMP_WALL)
                    )
                ), ParallelActions.ExitType.AND
            )
        )

        actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_CLAMP))

        actions.add(
            ParallelActions(
                arrayOf(
                    arrayListOf(
                        FollowRRTrajectory(
                            eventBus, newRRTrajectory(getEndOrientation(actions))
                                .setTangent(toRadians(0.0))
                                .splineToConstantHeading(Vector2d(9.2, 91.2), toRadians(-90.0), accelConstraintOverride = layerAccelConstrain)
                                .build()
                        )
                    ),
                    arrayListOf(
                        LiftAction(eventBus, IntakeManager.LiftPosition.UP_LAYER)
                    )
                ), ParallelActions.ExitType.AND
            )
        )

        actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_UNCLAMP))

        actions.add(
            ParallelActions(
                arrayOf(
                    arrayListOf(
                        FollowRRTrajectory(
                            eventBus, newRRTrajectory(getEndOrientation(actions))
                                .strafeToLinearHeading(
                                    Vector2d(-87.1, 149.1),
                                    toRadians(90.0)
                                )
                                .build()
                        )
                    ),
                    arrayListOf(
                        LiftAction(eventBus, IntakeManager.LiftPosition.TRANSPORT),
                        LiftAction(eventBus, IntakeManager.LiftPosition.CLAMP_WALL)
                    )
                ), ParallelActions.ExitType.AND
            )
        )

        actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_CLAMP))

        actions.add(
            ParallelActions(
                arrayOf(
                    arrayListOf(
                        FollowRRTrajectory(
                            eventBus, newRRTrajectory(getEndOrientation(actions))
                                .setTangent(toRadians(0.0))
                                .splineToConstantHeading(
                                    Vector2d(8.1, 97.8),
                                    toRadians(-90.0), accelConstraintOverride = layerAccelConstrain
                                )
                                .build()
                        )
                    ),
                    arrayListOf(
                        LiftAction(eventBus, IntakeManager.LiftPosition.UP_LAYER)
                    )
                ), ParallelActions.ExitType.AND
            )
        )

        actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_UNCLAMP))

        actions.add(
            ParallelActions(
                arrayOf(
                    arrayListOf(
                        FollowRRTrajectory(
                            eventBus, newRRTrajectory(getEndOrientation(actions))
                                .strafeToLinearHeading(
                                    Vector2d(-42.0, 140.4),
                                    toRadians(90.0 + 45.0)
                                )
                                .build()
                        )
                    ),
                    arrayListOf(
                        LiftAction(eventBus, IntakeManager.LiftPosition.CLAMP_CENTER, 1000.0)
                    )
                ), ParallelActions.ExitType.AND
            )
        )

        eventBus.invoke(RunActionsEvent(actions))
    }
}
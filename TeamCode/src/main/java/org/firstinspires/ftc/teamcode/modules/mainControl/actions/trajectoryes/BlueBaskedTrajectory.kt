//package org.firstinspires.ftc.teamcode.modules.mainControl.actions.trajectoryes
//
//import com.acmerobotics.roadrunner.Vector2d
//import org.firstinspires.ftc.teamcode.collectors.events.EventBus
//import org.firstinspires.ftc.teamcode.modules.intake.Intake
//import org.firstinspires.ftc.teamcode.modules.intake.IntakeManager
//import org.firstinspires.ftc.teamcode.modules.mainControl.actions.ActionsRunner
//import org.firstinspires.ftc.teamcode.modules.mainControl.actions.ClampAction
//import org.firstinspires.ftc.teamcode.modules.mainControl.actions.FollowRRTrajectory
//import org.firstinspires.ftc.teamcode.modules.mainControl.actions.IAction
//import org.firstinspires.ftc.teamcode.modules.mainControl.actions.ITransportAction.Companion.getEndOrientation
//import org.firstinspires.ftc.teamcode.modules.mainControl.actions.LiftAction
//import org.firstinspires.ftc.teamcode.modules.mainControl.actions.TurnAction
//import org.firstinspires.ftc.teamcode.modules.mainControl.actions.WaitAction
//import org.firstinspires.ftc.teamcode.modules.mainControl.actions.WaitLiftAction
//import org.firstinspires.ftc.teamcode.modules.mainControl.runner.TrajectorySegmentRunner.Companion.newRRTrajectory
//import org.firstinspires.ftc.teamcode.utils.units.Angle
//import org.firstinspires.ftc.teamcode.utils.units.Orientation
//import java.lang.Math.toRadians
//
//class BlueBaskedTrajectory : ITrajectoryBuilder {
//    override fun runTrajectory(eventBus: EventBus, startOrientation: Orientation) {
//        val actions = arrayListOf<IAction>()
//
//        /*        actions.add(FollowRRTrajectory(eventBus, newRRTrajectory(startOrientation)
//                    .strafeToLinearHeading(Vector2d(0.0, 80.0), toRadians(90.0))
//                    .build()))
//
//                actions.add(LiftAction(eventBus, IntakeManager.LiftPosition.UP_LAYER))
//
//                actions.add(WaitAction(2.0))
//
//                actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_UNCLAMP))
//
//                actions.add(WaitAction(2.0))
//        */
//        fun runToBasket(startOrientation: Orientation) {
//            actions.add(
//                FollowRRTrajectory(
//                    eventBus, newRRTrajectory(startOrientation)
//                        .strafeToLinearHeading(Vector2d(146.0, 126.0), toRadians(-90.0 - 45.0))
//                        .build()
//                )
//            )
//        }
//
//        fun basket() {
//            actions.add(LiftAction(eventBus, IntakeManager.LiftPosition.UP_BASKED))
//
//            actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_UNCLAMP))
//
//            actions.add(WaitLiftAction(eventBus))
//        }
//
//        fun clampStick(extension: Double){
//            actions.add(LiftAction(eventBus, IntakeManager.LiftPosition.CLAMP_CENTER, extension))
//
//            actions.add(WaitLiftAction(eventBus))
//
//            actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_CLAMP))
//
//            actions.add(WaitLiftAction(eventBus))
//        }
//
//        runToBasket(startOrientation)
//        basket()
//
//        actions.add(TurnAction(eventBus, getEndOrientation(actions), Angle.ofDeg(-90.0 - 10.0)))
//
//        clampStick(950.0)
//
//        actions.add(TurnAction(eventBus, getEndOrientation(actions), Angle.ofDeg(-90.0 - 45.0)))
//
//        basket()
//
//        actions.add(
//            FollowRRTrajectory(
//                eventBus, newRRTrajectory(getEndOrientation(actions))
//                    .strafeToLinearHeading(Vector2d(151.8, 126.0), toRadians(-90.0))
//                    .build()
//            )
//        )
//
//        clampStick(900.0)
//        runToBasket(getEndOrientation(actions))
//        basket()
//
//        eventBus.invoke(ActionsRunner.RunActionsEvent(actions))
//    }
//}

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

        /*        actions.add(FollowRRTrajectory(eventBus, newRRTrajectory(startOrientation)
                    .strafeToLinearHeading(Vector2d(0.0, 80.0), toRadians(90.0))
                    .build()))

                actions.add(LiftAction(eventBus, IntakeManager.LiftPosition.UP_LAYER))

                actions.add(WaitAction(2.0))

                actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_UNCLAMP))

                actions.add(WaitAction(2.0))
        */
        fun runToBasket(startOrientation: Orientation) {
            actions.add(
                FollowRRTrajectory(
                    eventBus, newRRTrajectory(startOrientation)
                        .strafeToLinearHeading(Vector2d(132.7, 132.7), toRadians(-90.0 - 45.0))
                        .build()
                )
            )
        }

        fun basket() {
            actions.add(LiftAction(eventBus, IntakeManager.LiftPosition.UP_BASKED))

            actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_UNCLAMP))

            actions.add(WaitLiftAction(eventBus))
        }

        fun clampStick(extension: Double){
            actions.add(LiftAction(eventBus, IntakeManager.LiftPosition.CLAMP_CENTER, extension))

            actions.add(WaitLiftAction(eventBus))

            actions.add(ClampAction(eventBus, Intake.ClampPosition.SERVO_CLAMP))

            actions.add(WaitAction(0.1))

            actions.add(WaitLiftAction(eventBus))
        }

        runToBasket(startOrientation)
        basket()

        actions.add(TurnAction(eventBus, getEndOrientation(actions), Angle.ofDeg(-90.0 - 12.0)))

        clampStick(1000.0)

        actions.add(TurnAction(eventBus, getEndOrientation(actions), Angle.ofDeg(-90.0 - 45.0)))

        basket()

        actions.add(
            FollowRRTrajectory(
                eventBus, newRRTrajectory(getEndOrientation(actions))
                    .strafeToLinearHeading(Vector2d(137.4, 122.3), toRadians(-90.0))
                    .build()
            )
        )

        clampStick(650.0)
        runToBasket(getEndOrientation(actions))
        basket()
//
//        actions.add(
//            FollowRRTrajectory(
//                eventBus, newRRTrajectory(getEndOrientation(actions))
//                    .strafeToLinearHeading(Vector2d(146.8, 121.0), toRadians(-65.0))
//                    .build()
//            )
//        )
//
//        clampStick(600.0)
//        runToBasket(getEndOrientation(actions), Vec2(-1.5))
//        basket()

        eventBus.invoke(ActionsRunner.RunActionsEvent(actions))
    }
}
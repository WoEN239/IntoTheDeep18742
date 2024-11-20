package org.firstinspires.ftc.teamcode.modules.mainControl.runner

import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.ProfileAccelConstraint
import com.acmerobotics.roadrunner.ProfileParams
import com.acmerobotics.roadrunner.Trajectory
import com.acmerobotics.roadrunner.TrajectoryBuilder
import com.acmerobotics.roadrunner.TrajectoryBuilderParams
import com.acmerobotics.roadrunner.TranslationalVelConstraint
import com.acmerobotics.roadrunner.Vector2d
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.modules.driveTrain.DriveTrain
import org.firstinspires.ftc.teamcode.modules.navigation.gyro.MergeGyro
import org.firstinspires.ftc.teamcode.modules.navigation.odometry.MergeOdometry
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.timer.ElapsedTimeExtra
import org.firstinspires.ftc.teamcode.utils.timer.Timers
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Vec2
import kotlin.math.abs

class ActionRunner : IRobotModule {
    class NewActionsEvent(var builder: ActionsBuilder?): IEvent
    class NewRRBuilder(var builder: TrajectoryBuilder?): IEvent

    private lateinit var _eventBus: EventBus

    private var _odometerPosition = Vec2.ZERO
    private var _gyroRotation = Angle.ZERO

    private var _odometerVelocity = Vec2.ZERO
    private var _gyroVelocity = 0.0

    private var _currentActions = arrayListOf<Action>()

    class RunActionsEvent(val trajectory: ActionsBuilder) : IEvent
    class EndTrajectoryEvent : IEvent

    private val _trajectoryTime = ElapsedTimeExtra()

    private var _targetHeadingVelocity = 0.0
    private var _targetTransVelocity = Vec2.ZERO

    private var _targetHeading = Angle.ZERO
    private var _targetPosition = Vec2.ZERO

    private var _currentActionAngle = Angle.ZERO
    private var _currentActionPosition = Vec2.ZERO

    override fun init(collector: BaseCollector, bus: EventBus) {
        _eventBus = bus

        bus.subscribe(RunActionsEvent::class) {
            if (_currentActions.isEmpty)
                _trajectoryTime.reset()

            _currentActions.addAll(it.trajectory.actions)

            _currentActionPosition = it.trajectory.currentPosition
            _currentActionAngle = it.trajectory.currentHeading
        }

        bus.subscribe(MergeOdometry.UpdateMergeOdometryEvent::class) {
            _odometerPosition = it.position
            _odometerVelocity = it.velocity
        }

        bus.subscribe(MergeGyro.UpdateMergeGyroEvent::class) {
            _gyroRotation = it.rotation
            _gyroVelocity = it.velocity
        }

        bus.subscribe(NewActionsEvent::class){
            it.builder = ActionsBuilder(_currentActionPosition, _currentActionAngle)
        }
        
        bus.subscribe(NewRRBuilder::class) {
            it.builder =
                TrajectoryBuilder(
                    TrajectoryBuilderParams(1e-6, ProfileParams(0.1, 0.1, 0.1)),
                    Pose2d(_currentActionPosition.x, _currentActionPosition.y, _currentActionAngle.angle), 0.0,
                    MinVelConstraint(
                        listOf(
                            TranslationalVelConstraint(Configs.RoadRunnerConfig.MAX_TRANSLATION_VELOCITY),
                            AngularVelConstraint(Configs.RoadRunnerConfig.MAX_ROTATE_VELOCITY)
                        )
                    ),
                    ProfileAccelConstraint(
                        -Configs.RoadRunnerConfig.MAX_TRANSLATION_ACCEL,
                        Configs.RoadRunnerConfig.MAX_TRANSLATION_ACCEL
                    )
                )
        }
    }

    override fun start() {
        ActionRunnerHelper.runActions(ActionRunnerHelper.newAB()
            .runRRTrajectory(ActionRunnerHelper.newTB().strafeTo(Vector2d(40.0, 40.0))).turn(Angle(Math.toRadians(90.0))))
    }

    override fun update() {
        val headingErr = (_targetHeading - _gyroRotation).angle
        val posErr = (_targetPosition - _odometerPosition).turn(-_gyroRotation.angle)

        _eventBus.invoke(
            DriveTrain.SetDriveCmEvent(
                _targetTransVelocity +
                        if(abs(posErr.x) > Configs.RoadRunnerConfig.POSITION_SENS_X) Vec2(posErr.x * Configs.RoadRunnerConfig.POSITION_P_X, 0.0) else Vec2.ZERO +
                        if(abs(posErr.y) > Configs.RoadRunnerConfig.POSITION_SENS_Y) Vec2(0.0, -posErr.y * Configs.RoadRunnerConfig.POSITION_P_Y) else Vec2.ZERO,
                _targetHeadingVelocity + if(abs(headingErr) > Configs.RoadRunnerConfig.ROTATE_SENS) headingErr * Configs.RoadRunnerConfig.ROTATE_P else 0.0
            )
        )

        StaticTelemetry.addData("err", posErr)
        StaticTelemetry.addData("targetPosition", _targetPosition)

        if (_currentActions.isEmpty)
            return

        val trajectory = _currentActions[0]

        val time = _trajectoryTime.seconds()

        _targetHeadingVelocity = trajectory.turnVelocity(time)

        _targetTransVelocity = trajectory.transVelocity(time)

        _targetHeading = trajectory.targetHeading(time)

        _targetPosition = trajectory.targetPosition(time) * Vec2(1.0, -1.0)

        if(trajectory.isEnd(time)){
            _currentActions.removeAt(0)

            _trajectoryTime.reset()

            _eventBus.invoke(EndTrajectoryEvent())
        }
    }

    class ActionsBuilder(var currentPosition: Vec2, var currentHeading: Angle) {
        val actions = arrayListOf<Action>()

        fun turn(rot: Angle): ActionsBuilder {
            val action = Turn(rot.angle, currentHeading, currentPosition)

            actions.add(action)

            val pos = action.getEndPosition(currentHeading, currentPosition)

            currentPosition = pos.second
            currentHeading = pos.first

            return this
        }

        fun runRRTrajectory(trajectory: TrajectoryBuilder) = runRRBuildedTrajectory(trajectory.build())

        fun runRRBuildedTrajectory(build: List<Trajectory>): ActionsBuilder {
            val action = RunBuildedTrajectory(build)

            actions.add(action)

            val pos = action.getEndPosition(currentHeading, currentPosition)

            currentPosition = pos.second
            currentHeading = pos.first

            return this
        }
    }
}
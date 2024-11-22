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
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Orientation
import org.firstinspires.ftc.teamcode.utils.units.Vec2
import kotlin.math.abs

class TrajectoryRunner : IRobotModule {
    class NewTrajectoryEvent(var builder: TrajectoryActionBuilder?): IEvent
    class NewRRBuilder(var builder: TrajectoryBuilder?): IEvent

    private lateinit var _eventBus: EventBus

    private var _currentTrajectory = arrayListOf<TrajectorySegment>()

    class RunTrajectoryEvent(val trajectory: TrajectoryActionBuilder) : IEvent
    class EndTrajectoryEvent : IEvent

    private val _trajectoryTime = ElapsedTimeExtra()

    private var _targetHeadingVelocity = 0.0
    private var _targetTransVelocity = Vec2.ZERO

    private var _targetOrientation = Orientation.ZERO

    private var _currentTrajectoryOrientation = Orientation.ZERO

    override fun init(collector: BaseCollector, bus: EventBus) {
        _eventBus = bus

        bus.subscribe(RunTrajectoryEvent::class) {
            if (_currentTrajectory.isEmpty)
                _trajectoryTime.reset()

            _currentTrajectory.addAll(it.trajectory.actions)

            _currentTrajectoryOrientation = it.trajectory.currentOrientation
        }

        bus.subscribe(NewTrajectoryEvent::class){
            it.builder = TrajectoryActionBuilder(_currentTrajectoryOrientation)
        }
        
        bus.subscribe(NewRRBuilder::class) {
            it.builder =
                TrajectoryBuilder(
                    TrajectoryBuilderParams(1e-6, ProfileParams(0.1, 0.1, 0.1)),
                    Pose2d(_currentTrajectoryOrientation.x, _currentTrajectoryOrientation.y, _currentTrajectoryOrientation.angl.angle), 0.0,
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

    override fun update() {
        val gyro = MergeGyro.RequestMergeRotateEvent()
        _eventBus.invoke(gyro)

        val odometry = MergeOdometry.RequestMergePositionEvent()
        _eventBus.invoke(odometry)

        val headingErr = (_targetOrientation.angl - gyro.rotation!!).angle
        val posErr = (_targetOrientation.pos - odometry.position!!).turn(-gyro.rotation!!.angle)

        _eventBus.invoke(
            DriveTrain.SetDriveCmEvent(
                _targetTransVelocity +
                        if(abs(posErr.x) > Configs.RoadRunnerConfig.POSITION_SENS_X) Vec2(posErr.x * Configs.RoadRunnerConfig.POSITION_P_X, 0.0) else Vec2.ZERO +
                        if(abs(posErr.y) > Configs.RoadRunnerConfig.POSITION_SENS_Y) Vec2(0.0, posErr.y * Configs.RoadRunnerConfig.POSITION_P_Y) else Vec2.ZERO,
                _targetHeadingVelocity + if(abs(headingErr) > Configs.RoadRunnerConfig.ROTATE_SENS) headingErr * Configs.RoadRunnerConfig.ROTATE_P else 0.0
            )
        )

        StaticTelemetry.addData("err", posErr)
        StaticTelemetry.addData("targetPosition", _targetOrientation.pos)

        if (_currentTrajectory.isEmpty)
            return

        val trajectory = _currentTrajectory[0]

        val time = _trajectoryTime.seconds()

        _targetHeadingVelocity = trajectory.turnVelocity(time)

        _targetTransVelocity = trajectory.transVelocity(time)

        _targetOrientation = trajectory.targetOrientation(time) * Orientation(Vec2(1.0, -1.0))

        if(trajectory.isEnd(time)){
            _currentTrajectory.removeAt(0)

            _trajectoryTime.reset()

            _eventBus.invoke(EndTrajectoryEvent())
        }
    }

    class TrajectoryActionBuilder(var currentOrientation: Orientation) {
        val actions = arrayListOf<TrajectorySegment>()

        fun turn(rot: Angle): TrajectoryActionBuilder {
            val action = Turn(rot.angle, currentOrientation)

            actions.add(action)

            currentOrientation = action.getEndOrientation(currentOrientation)

            return this
        }

        fun runRRTrajectory(trajectory: TrajectoryBuilder) = runRRBuildedTrajectory(trajectory.build())

        fun runRRBuildedTrajectory(build: List<Trajectory>): TrajectoryActionBuilder {
            val action = RunBuildedTrajectory(build)

            actions.add(action)

            currentOrientation = action.getEndOrientation(currentOrientation)

            return this
        }
    }
}
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
import org.firstinspires.ftc.teamcode.utils.units.Color
import org.firstinspires.ftc.teamcode.utils.units.Orientation
import org.firstinspires.ftc.teamcode.utils.units.Vec2
import kotlin.math.abs

class TrajectoryRunner : IRobotModule {
    class NewTrajectoryEvent(var builder: TrajectoryActionBuilder? = null): IEvent
    class NewRRBuilder(val startPosition: Orientation, var builder: TrajectoryBuilder? = null): IEvent

    private lateinit var _eventBus: EventBus

    private var _currentTrajectory = arrayListOf<TrajectorySegment>()

    class RunTrajectoryEvent(val trajectory: TrajectoryActionBuilder) : IEvent
    class RequestIsEndTrajectoryEvent(var isEnd: Boolean = false) : IEvent

    private val _trajectoryTime = ElapsedTimeExtra()

    private var _targetHeadingVelocity = 0.0
    private var _targetTransVelocity = Vec2.ZERO

    private var _targetOrientation = Orientation.ZERO

    private var _currentTrajectoryOrientation = Orientation.ZERO

    override fun init(collector: BaseCollector, bus: EventBus) {
        _eventBus = bus

        bus.subscribe(RequestIsEndTrajectoryEvent::class){
            it.isEnd = _currentTrajectory.isEmpty
        }

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
                    Pose2d(it.startPosition.x, it.startPosition.y, it.startPosition.angl.angle), 0.0,
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
        Timers.newTimer().start(8.0) {
            val trajectory = NewTrajectoryEvent()
            val rrBuilder = NewRRBuilder(Orientation.ZERO)
            val rrBuilder2 = NewRRBuilder(Orientation(Vec2(20.0, 20.0), Angle.ofDeg(90.0)))

            _eventBus.invoke(trajectory)
            _eventBus.invoke(rrBuilder)
            _eventBus.invoke(rrBuilder2)

            val builder = rrBuilder.builder!!.splineTo(Vector2d(20.0, 20.0), Math.toRadians(-90.0))
            val builder2 = rrBuilder2.builder!!.strafeTo(Vector2d(0.0, 0.0))

            _eventBus.invoke(RunTrajectoryEvent(trajectory.builder!!
                .runRRTrajectory(builder)
                //.turn(Math.toRadians(90.0))
                /*.runRRTrajectory(builder2)*/))
        }
    }

    override fun update() {
        val gyro = MergeGyro.RequestMergeRotateEvent()
        _eventBus.invoke(gyro)

        val odometry = MergeOdometry.RequestMergePositionEvent()
        _eventBus.invoke(odometry)

        val headingErr = (_targetOrientation.angl - gyro.rotation!!).angle
        val posErr = (_targetOrientation.pos - odometry.position!!)

        _eventBus.invoke(
            DriveTrain.SetDriveCmEvent(
                _targetTransVelocity.turn(gyro.rotation!!.angle) +
                        (if(abs(posErr.x) > Configs.RoadRunnerConfig.POSITION_SENS_X) Vec2(posErr.x * Configs.RoadRunnerConfig.POSITION_P_X, 0.0) else Vec2.ZERO +
                        if(abs(posErr.y) > Configs.RoadRunnerConfig.POSITION_SENS_Y) Vec2(0.0, posErr.y * Configs.RoadRunnerConfig.POSITION_P_Y) else Vec2.ZERO).turn(gyro.rotation!!.angle),
                _targetHeadingVelocity + if(abs(headingErr) > Configs.RoadRunnerConfig.ROTATE_SENS) headingErr * Configs.RoadRunnerConfig.ROTATE_P else 0.0
            )
        )

        StaticTelemetry.drawRect(_targetOrientation.pos, Vec2(20.0, 20.0), _targetOrientation.angl.angle, Color.ORANGE)
        StaticTelemetry.addData("targetPosition", _targetOrientation.pos)

        if (_currentTrajectory.isEmpty)
            return

        val trajectory = _currentTrajectory[0]

        val time = _trajectoryTime.seconds()

        _targetHeadingVelocity = trajectory.turnVelocity(time)

        _targetTransVelocity = trajectory.transVelocity(time)

        _targetOrientation = trajectory.targetOrientation(time)

        if(trajectory.isEnd(time)){
            _currentTrajectory.removeAt(0)

            _targetTransVelocity = Vec2.ZERO
            _targetHeadingVelocity = 0.0

            _trajectoryTime.reset()
        }
    }

    class TrajectoryActionBuilder(var currentOrientation: Orientation) {
        val actions = arrayListOf<TrajectorySegment>()

        fun turn(rot: Double): TrajectoryActionBuilder {
            val action = Turn(rot, currentOrientation)

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
package org.firstinspires.ftc.teamcode.modules.mainControl.runner

import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.Trajectory
import com.acmerobotics.roadrunner.TrajectoryBuilder
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.modules.driveTrain.DriveTrain
import org.firstinspires.ftc.teamcode.modules.navigation.gyro.MergeGyro
import org.firstinspires.ftc.teamcode.modules.navigation.odometry.MergeOdometry
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.timer.ElapsedTimeExtra
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Vec2

class RoadRunner : IRobotModule {
    class NewActionsEvent(var builder: ActionsBuilder?): IEvent

    private lateinit var _eventBus: EventBus

    private var _odometerPosition = Vec2.ZERO
    private var _gyroRotation = Angle.ZERO

    private var _odometerVelocity = Vec2.ZERO
    private var _gyroVelocity = 0.0

    private var _currentActions = arrayListOf<Action>()

    class RunTrajectoryEvent(val trajectory: ActionsBuilder) : IEvent
    class EndTrajectoryEvent : IEvent

    private val _trajectoryTime = ElapsedTimeExtra()

    private var _targetHeadingVelocity = 0.0
    private var _targetTransVelocity = Vec2.ZERO

    private var _targetHeading = Angle.ZERO
    private var _targetPosition = Vec2.ZERO

    override fun init(collector: BaseCollector, bus: EventBus) {
        _eventBus = bus

        bus.subscribe(RunTrajectoryEvent::class) {
            if (_currentActions.isEmpty())
                _trajectoryTime.reset()

            _currentActions = it.trajectory.actions
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
            it.builder = ActionsBuilder(_targetPosition, _targetHeading)
        }
    }

    override fun start() {

    }

    override fun update() {
        _eventBus.invoke(
            DriveTrain.SetDriveCmEvent(
                _targetTransVelocity +
                        (_targetTransVelocity - _odometerVelocity) * Vec2(Configs.RoadRunnerConfig.POSITION_VELOCITY_P) +
                        (_targetPosition - _odometerPosition) * Vec2(Configs.RoadRunnerConfig.POSITION_P),
                _targetHeadingVelocity +
                        (_targetHeadingVelocity - _gyroVelocity) * Configs.RoadRunnerConfig.ROTATE_VELOCITY_P +
                        (_targetHeading - _gyroRotation).angle * Configs.RoadRunnerConfig.ROTATE_P
            )
        )

        if (_currentActions.isEmpty())
            return

        val trajectory = _currentActions[0]

        val time = _trajectoryTime.seconds()

        _targetHeadingVelocity = trajectory.turnVelocity(time)

        _targetTransVelocity = trajectory.transVelocity(time)

        _targetHeading = trajectory.targetHeading(time)

        _targetPosition = trajectory.targetPosition(time)

        if(trajectory.isEnd(time)){
            _currentActions.removeAt(0)

            _trajectoryTime.reset()

            _eventBus.invoke(EndTrajectoryEvent())
        }
    }

    class ActionsBuilder(var currentPosition: Vec2, var currentHeading: Angle) {
        val actions = arrayListOf<Action>()

        fun turnTo(rot: Angle): ActionsBuilder {
            actions.add(TurnTo(rot.angle, currentPosition))

            currentHeading = rot

            return this
        }

        fun runRRTrajectory(trajectory: TrajectoryBuilder) = runRRBuildedTrajectory(trajectory.build())

        fun runRRBuildedTrajectory(build: List<Trajectory>): ActionsBuilder {
            val action = RunBuildedTrajectory(build)

            currentPosition = action.targetPosition(action.duration())
            currentHeading = action.targetHeading(action.duration())

            actions.add(action)

            return this
        }
    }
}
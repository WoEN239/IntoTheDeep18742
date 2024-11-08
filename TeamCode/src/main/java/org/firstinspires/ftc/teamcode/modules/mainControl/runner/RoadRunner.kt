package org.firstinspires.ftc.teamcode.modules.mainControl.runner

import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.ProfileAccelConstraint
import com.acmerobotics.roadrunner.ProfileParams
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.Trajectory
import com.acmerobotics.roadrunner.TrajectoryBuilder
import com.acmerobotics.roadrunner.TrajectoryBuilderParams
import com.acmerobotics.roadrunner.TranslationalVelConstraint
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.modules.driveTrain.DriveTrain
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.timer.ElapsedTimeExtra
import org.firstinspires.ftc.teamcode.utils.units.Vec2
import java.util.concurrent.Callable
import java.util.concurrent.Executors

class RoadRunner : IRobotModule {
    private lateinit var _robot: LinearOpMode

    private lateinit var _eventBus: EventBus

    override fun init(collector: BaseCollector, bus: EventBus) {
        _robot = collector.robot
        _eventBus = bus
    }

    override fun start() {
        runTrajectory(newTrajectory.turnTo(Math.toRadians(90.0)))
    }

    override fun update() {
        val currentTrajectory = _currentTrajectory[0]

        updateTrajectory(currentTrajectory, _trajectoryTime.seconds())
    }

    private var _currentTrajectory = arrayListOf<Any>()

    val newTrajectory
        get() = ThreadedTrajectoryBuilder(Configs.RoadRunnerConfig.BUILDER_THREAD_COUNT)

    fun runTrajectory(trajectory: ThreadedTrajectoryBuilder) {
        if (_currentTrajectory.isEmpty())
            _trajectoryTime.reset()

        for (i in trajectory.build())
            _currentTrajectory.add(if (i is Trajectory) TimeTrajectory(i) else i)

        pause = false
    }

    private val _trajectoryTime = ElapsedTimeExtra()

    val isEndTrajectory: Boolean
        get() = _currentTrajectory.isEmpty()

    private fun updateTrajectory(trajectory: Any, time: Double){
        if (pause)
            return

        val headingVelocity =
            when (trajectory) {
                is TimeTrajectory -> trajectory[time].velocity().angVel.value()
                is Action -> trajectory.turnVelocity(time)
                else -> throw Exception("trajectory not support " + trajectory::class.simpleName)
            }

        val transVelocity =
            when (trajectory) {
                is TimeTrajectory -> Vec2(trajectory[time].velocity().linearVel.value())
                is Action -> trajectory.transVelocity(time)
                else -> throw Exception("trajectory not support " + trajectory::class.simpleName)
            }

        _eventBus.invoke(DriveTrain.SetDriveCmEvent(transVelocity, headingVelocity))

        if ((trajectory is TimeTrajectory && time > trajectory.duration) || (trajectory is Action && trajectory.isEnd())) {
            _currentTrajectory.removeAt(0)

            if (_currentTrajectory.isEmpty()) {
                pause = true

                _eventBus.invoke(DriveTrain.SetDriveCmEvent(Vec2.ZERO, 0.0))
            }
        }
    }

    var pause: Boolean = false
        set(value) {
            if (isEndTrajectory)
                return

            field = value

            if (!value) {
                updateTrajectory(_currentTrajectory[0], _trajectoryTime.seconds())

                _trajectoryTime.start()

                return
            }

            _trajectoryTime.pause()

            _eventBus.invoke(DriveTrain.SetDriveCmEvent(Vec2.ZERO, 0.0))
        }

    class ThreadedTrajectoryBuilder(
        builderThreadCount: Int,
        private var _oldPose: Pose2d = Pose2d(
            Vector2d(0.0, 0.0), 0.0
        )
    ) {
        private val _executorService = Executors.newWorkStealingPool(builderThreadCount)

        private val _trajectoryBuilders = mutableListOf<Any>();
        fun build(): List<Any> {
            val tasks = arrayListOf<Callable<Any>>()

            for (i in _trajectoryBuilders)
                tasks.add { if (i is TrajectoryBuilder) i.build()[0] else i }

            val threadResult = _executorService.invokeAll(tasks)

            val result = arrayListOf<Any>()

            for (i in threadResult) {
                while (!i.isDone);

                result.add(i.get())
            }

            return result
        }

        private fun newTB(begPose: Pose2d) =
            TrajectoryBuilder(
                TrajectoryBuilderParams(1e-6, ProfileParams(0.1, 0.1, 0.1)),
                begPose, 0.0,
                MinVelConstraint(
                    listOf(
                        TranslationalVelConstraint(Configs.RoadRunnerConfig.MAX_TRANSLATION_VELOCITY),
                        AngularVelConstraint(Configs.RoadRunnerConfig.MAX_ROTATE_VELOCITY)
                    )
                ),
                ProfileAccelConstraint(
                    -Configs.RoadRunnerConfig.MAX_ACCEL,
                    Configs.RoadRunnerConfig.MAX_ACCEL
                )
            )

        private fun last(): TrajectoryBuilder {
            val last = _trajectoryBuilders.last()

            if (last is TrajectoryBuilder)
                return last

            _trajectoryBuilders.add(newTB(_oldPose))

            return _trajectoryBuilders.last() as TrajectoryBuilder
        }

        fun splineTo(pos: Vec2, tangent: Double): ThreadedTrajectoryBuilder {
            last().splineTo(
                Vector2d(pos.x, pos.y), tangent
            )

            _oldPose = Pose2d(Vector2d(pos.x, pos.y), tangent)

            return this
        }

        fun splineToConstantHeading(
            pos: Vec2,
            tangent: Double
        ): ThreadedTrajectoryBuilder {
            last().splineToConstantHeading(
                Vector2d(pos.x, pos.y), tangent
            )

            _oldPose = Pose2d(Vector2d(pos.x, pos.y), tangent)

            return this
        }

        fun strafeToConstantHeading(
            pos: Vec2,
        ): ThreadedTrajectoryBuilder {
            last().strafeToConstantHeading(
                Vector2d(pos.x, pos.y)
            )

            _oldPose = Pose2d(Vector2d(pos.x, pos.y), _oldPose.heading)

            return this
        }

        fun strafeTo(
            pos: Vec2,
        ): ThreadedTrajectoryBuilder {
            last().strafeTo(
                Vector2d(pos.x, pos.y)
            )

            _oldPose = Pose2d(Vector2d(pos.x, pos.y), _oldPose.heading)

            return this
        }

        fun turnTo(rot: Double): ThreadedTrajectoryBuilder {
            _trajectoryBuilders.add(TurnTo(rot))

            return this
        }
    }
}
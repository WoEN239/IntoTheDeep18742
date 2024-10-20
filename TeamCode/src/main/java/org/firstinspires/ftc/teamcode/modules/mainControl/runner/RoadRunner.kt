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
import org.firstinspires.ftc.teamcode.modules.driveTrain.DriveTrain
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.timer.ElapsedTimeExtra
import org.firstinspires.ftc.teamcode.utils.units.Vec2
import java.util.concurrent.Callable
import java.util.concurrent.Executors

object RoadRunner : IRobotModule {
    private lateinit var _robot: LinearOpMode

    override fun init(collector: BaseCollector) {
        _robot = collector.robot
    }

    override fun update() {
        if (pause)
            return

        val currentTrajectory = _currentTrajectory[0]

        val headingVelocity =
            if (currentTrajectory is TimeTrajectory) currentTrajectory[_trajectoryTime.seconds()].velocity().angVel.value()
            else if (currentTrajectory is Action) currentTrajectory.turnVelocity(_trajectoryTime.seconds()) else 0.0

        val transVelocity =
            if (currentTrajectory is TimeTrajectory) Vec2(currentTrajectory[_trajectoryTime.seconds()].velocity().linearVel.value())
            else if (currentTrajectory is Action) currentTrajectory.transVelocity(_trajectoryTime.seconds()) else Vec2.ZERO

        DriveTrain.driveCmDirection(transVelocity, headingVelocity)

        if ((currentTrajectory is TimeTrajectory && _trajectoryTime.seconds() > currentTrajectory.duration) || (currentTrajectory is Action && currentTrajectory.isEnd())) {
            _currentTrajectory.removeAt(0)

            _trajectoryTime.reset()

            if (_currentTrajectory.isEmpty()) {
                pause = true

                DriveTrain.stop()
            }
        }
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

    var pause: Boolean = false
        set(value) {
            if (isEndTrajectory)
                return

            field = value

            if (!value) {
                val currentTrajectory = _currentTrajectory[0]

                val headingVelocity =
                    if (currentTrajectory is TimeTrajectory) currentTrajectory[_trajectoryTime.seconds()].velocity().angVel.value()
                    else if (currentTrajectory is Action) currentTrajectory.turnVelocity(
                        _trajectoryTime.seconds()
                    ) else 0.0

                val transVelocity =
                    if (currentTrajectory is TimeTrajectory) Vec2(currentTrajectory[_trajectoryTime.seconds()].velocity().linearVel.value())
                    else if (currentTrajectory is Action) currentTrajectory.transVelocity(
                        _trajectoryTime.seconds()
                    ) else Vec2.ZERO

                DriveTrain.driveCmDirection(transVelocity, headingVelocity)

                _trajectoryTime.start()

                return
            }

            _trajectoryTime.pause()
            DriveTrain.stop()
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
                while (!i.isDone)
                    if (!_robot.opModeIsActive()) {
                        _executorService.shutdown()

                        return emptyList()
                    }

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
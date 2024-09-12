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
        if(pause)
            return

        val headingVelocity =
            _currentTrajectory[0][_trajectoryTime.seconds()].velocity().angVel
        val transVelocity =
            _currentTrajectory[0][_trajectoryTime.seconds()].velocity().linearVel

        DriveTrain.driveCmDirection(
            Vec2(transVelocity.x.value(), transVelocity.y.value()),
            headingVelocity.value()
        )

        if(_trajectoryTime.seconds() > _currentTrajectory[0].duration){
            _currentTrajectory.removeAt(0)

            _trajectoryTime.reset()

            if(_currentTrajectory.isEmpty())
                pause = true
        }
    }

    private var _currentTrajectory = arrayListOf<TimeTrajectory>()

    val newTrajectory
            get() = ThreadedTrajectoryBuilder(Configs.RoadRunnerConfig.BUILDER_THREAD_COUNT)

    fun runTrajectory(trajectory: ThreadedTrajectoryBuilder){
        if(_currentTrajectory.isEmpty())
            _trajectoryTime.reset()

        for (i in trajectory.build())
            _currentTrajectory.add(TimeTrajectory(i))

        pause = false
    }

    private val _trajectoryTime = ElapsedTimeExtra()

    val endTrajectory: Boolean
        get() = _currentTrajectory.isEmpty()

    var pause: Boolean = false
        set(value) {
            if(endTrajectory)
                return

            field = value

            if(!value) {
                val headingVelocity =
                    _currentTrajectory[0][_trajectoryTime.seconds()].velocity().angVel
                val transVelocity =
                    _currentTrajectory[0][_trajectoryTime.seconds()].velocity().linearVel

                DriveTrain.driveCmDirection(
                    Vec2(transVelocity.x.value(), transVelocity.y.value()),
                    headingVelocity.value()
                )

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

        private val _trajectoryBuilders = mutableListOf<TrajectoryBuilder>()

        fun build(): List<Trajectory> {
            val tasks = arrayListOf<Callable<Trajectory>>()

            for (i in _trajectoryBuilders)
                tasks.add { i.build()[0] }

            val threadResult = _executorService.invokeAll(tasks)

            val result = arrayListOf<Trajectory>()

            for (i in threadResult) {
                while (!i.isDone)
                    if(!_robot.opModeIsActive()) {
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

        fun splineTo(beginPose: Pose2d, pos: Vec2, tangent: Double): ThreadedTrajectoryBuilder {
            _trajectoryBuilders.add(
                newTB(beginPose).splineTo(
                    Vector2d(pos.x, pos.y), tangent
                )
            )

            _oldPose = Pose2d(Vector2d(pos.x, pos.y), tangent)

            return this
        }

        fun splineTo(pos: Vec2, tangent: Double) = splineTo(_oldPose, pos, tangent)


        fun splineToConstantHeading(
            beginPose: Pose2d,
            pos: Vec2,
            tangent: Double
        ): ThreadedTrajectoryBuilder {
            _trajectoryBuilders.add(
                newTB(beginPose).splineToConstantHeading(
                    Vector2d(pos.x, pos.y), tangent
                )
            )

            _oldPose = Pose2d(Vector2d(pos.x, pos.y), tangent)

            return this
        }

        fun splineToConstantHeading(pos: Vec2, tangent: Double) =
            splineToConstantHeading(_oldPose, pos, tangent)

        fun strafeToConstantHeading(
            beginPose: Pose2d,
            pos: Vec2,
        ): ThreadedTrajectoryBuilder {
            _trajectoryBuilders.add(
                newTB(beginPose).strafeToConstantHeading(
                    Vector2d(pos.x, pos.y)
                )
            )

            _oldPose = Pose2d(Vector2d(pos.x, pos.y), _oldPose.heading)

            return this
        }

        fun strafeToConstantHeading(pos: Vec2) =
            strafeToConstantHeading(_oldPose, pos)

        fun strafeTo(
            beginPose: Pose2d,
            pos: Vec2,
        ): ThreadedTrajectoryBuilder {
            _trajectoryBuilders.add(
                newTB(beginPose).strafeTo(
                    Vector2d(pos.x, pos.y)
                )
            )

            _oldPose = Pose2d(Vector2d(pos.x, pos.y), _oldPose.heading)

            return this
        }

        fun strafeTo(pos: Vec2) =
            strafeTo(_oldPose, pos)
    }
}
package org.firstinspires.ftc.teamcode.modules.mainControl.runner

import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.HolonomicController
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.ProfileAccelConstraint
import com.acmerobotics.roadrunner.ProfileParams
import com.acmerobotics.roadrunner.TrajectoryBuilder
import com.acmerobotics.roadrunner.TrajectoryBuilderParams
import com.acmerobotics.roadrunner.TranslationalVelConstraint
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
import org.firstinspires.ftc.teamcode.utils.units.Color
import org.firstinspires.ftc.teamcode.utils.units.Orientation
import org.firstinspires.ftc.teamcode.utils.units.Vec2
import kotlin.math.abs

class TrajectorySegmentRunner : IRobotModule {
    companion object {
        fun newRRTrajectory(startOrientation: Orientation) = TrajectoryBuilder(
            TrajectoryBuilderParams(1e-6, ProfileParams(0.1, 0.1, 0.1)),
            Pose2d(startOrientation.x, startOrientation.y, startOrientation.angl.angle), 0.0,
            MinVelConstraint(
                listOf(
                    TranslationalVelConstraint(Configs.DriveTrainConfig.MAX_TRANSLATION_VELOCITY),
                    AngularVelConstraint(Configs.DriveTrainConfig.MAX_ROTATE_VELOCITY)
                )
            ),
            ProfileAccelConstraint(
                Configs.DriveTrainConfig.MIN_TRANSLATION_ACCEL,
                Configs.DriveTrainConfig.MAX_TRANSLATION_ACCEL
            )
        )
    }

    private lateinit var _eventBus: EventBus

    private var _currentTrajectory = arrayListOf<ITrajectorySegment>()

    class RunTrajectorySegmentEvent(val trajectory: ITrajectorySegment) : IEvent
    class RequestIsEndTrajectoryEvent(var isEnd: Boolean = false) : IEvent

    private val _trajectoryTime = ElapsedTimeExtra()

    private var _targetHeadingVelocity = 0.0
    private var _targetTransVelocity = Vec2.ZERO

    private var _targetOrientation = Orientation.ZERO

    private var _posErr = Vec2.ZERO
    private var _headingErr = 0.0

    override fun init(collector: BaseCollector, bus: EventBus) {
        _targetOrientation = Orientation(collector.parameters.oldStartPosition.position, collector.parameters.oldStartPosition.angle)

        _eventBus = bus

        bus.subscribe(RequestIsEndTrajectoryEvent::class) {
            it.isEnd = _currentTrajectory.isEmpty() &&
                    abs(_posErr.x) < Configs.RoadRunnerConfig.POSITION_SENS_X + Configs.RoadRunnerConfig.STEP_X &&
                    abs(_posErr.y) < Configs.RoadRunnerConfig.POSITION_SENS_Y + Configs.RoadRunnerConfig.STEP_Y &&
                    abs(_headingErr) < Configs.RoadRunnerConfig.ROTATE_SENS + Configs.RoadRunnerConfig.STEP_H
        }

        bus.subscribe(RunTrajectorySegmentEvent::class) {
            if (_currentTrajectory.isEmpty())
                _trajectoryTime.reset()

            _currentTrajectory.add(it.trajectory)
        }
    }

    override fun update() {
        val gyro = _eventBus.invoke(MergeGyro.RequestMergeGyroEvent())
        val odometry = _eventBus.invoke(MergeOdometry.RequestMergePositionEvent())

        val localizedTransVelocity = _targetTransVelocity.turn(
            -gyro.rotation!!.angle
        )

        _headingErr = (_targetOrientation.angl - gyro.rotation!!).angle
        _posErr = (_targetOrientation.pos - odometry.position!!).turn(-gyro.rotation!!.angle)

        val velPosErr = localizedTransVelocity - odometry.velocity!!
        val velHeadingErr = _targetHeadingVelocity - gyro.velocity!!

        val uPos =
            Vec2(
                if (abs(_posErr.x) > Configs.RoadRunnerConfig.POSITION_SENS_X) _posErr.x * Configs.RoadRunnerConfig.POSITION_P_X else 0.0,
                if (abs(_posErr.y) > Configs.RoadRunnerConfig.POSITION_SENS_Y) _posErr.y * Configs.RoadRunnerConfig.POSITION_P_Y else 0.0
            )

        val uPosVel = Vec2(
            if (abs(velPosErr.x) > Configs.RoadRunnerConfig.POS_VELOCITY_SENS_X) velPosErr.x * Configs.RoadRunnerConfig.POS_VELOCITY_P_X else 0.0,
            if (abs(velPosErr.y) > Configs.RoadRunnerConfig.POS_VELOCITY_SENS_Y) velPosErr.y * Configs.RoadRunnerConfig.POS_VELOCITY_P_Y else 0.0)

        val headingU =
            if (abs(_headingErr) > Configs.RoadRunnerConfig.ROTATE_SENS) _headingErr * Configs.RoadRunnerConfig.ROTATE_P else 0.0

        val headingVelU =
            if (abs(velHeadingErr) > Configs.RoadRunnerConfig.HEADING_VEL_SENS) velHeadingErr * Configs.RoadRunnerConfig.HEADING_VEL_P else 0.0

        _eventBus.invoke(
            DriveTrain.SetDriveCmEvent(
                localizedTransVelocity + uPos + uPosVel,
                _targetHeadingVelocity + headingU + headingVelU
            )
        )

        StaticTelemetry.drawRect(
            _targetOrientation.pos,
            Vec2(20.0, 20.0),
            _targetOrientation.angl.angle,
            Color.ORANGE
        )
        StaticTelemetry.addData("targetPosition", _targetOrientation.pos)

        StaticTelemetry.addData("segments count", _currentTrajectory.size)

        if (_currentTrajectory.isEmpty())
            return

        val trajectory = _currentTrajectory[0]

        val time = _trajectoryTime.seconds()

        _targetHeadingVelocity = trajectory.turnVelocity(time)

        _targetTransVelocity = trajectory.transVelocity(time) * Vec2(0.25)

        _targetOrientation = trajectory.targetOrientation(time)

        if (trajectory.isEnd(time)) {
            _currentTrajectory.removeAt(0)

            _targetTransVelocity = Vec2.ZERO
            _targetHeadingVelocity = 0.0

            _trajectoryTime.reset()
        }
    }
}
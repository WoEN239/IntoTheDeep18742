package org.firstinspires.ftc.teamcode.modules.mainControl.runner

import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Pose2dDual
import com.acmerobotics.roadrunner.ProfileAccelConstraint
import com.acmerobotics.roadrunner.ProfileParams
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.TimeTurn
import com.acmerobotics.roadrunner.Trajectory
import com.acmerobotics.roadrunner.TrajectoryBuilder
import com.acmerobotics.roadrunner.TrajectoryBuilderParams
import com.acmerobotics.roadrunner.TranslationalVelConstraint
import com.acmerobotics.roadrunner.TurnConstraints
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Vec2

interface Action {
    fun isEnd(time: Double): Boolean

    fun transVelocity(time: Double): Vec2

    fun turnVelocity(time: Double): Double

    fun targetHeading(time: Double): Angle

    fun targetPosition(time: Double): Vec2
}

class TurnTo(val angle: Double, currentAngle: Angle, val currentPosition: Vec2): Action{
    private val _turn = TimeTurn(Pose2d(currentPosition.x, currentPosition.y, currentAngle.angle), angle,
        TurnConstraints(Configs.RoadRunnerConfig.MAX_ROTATE_VELOCITY, -Configs.RoadRunnerConfig.ROTATE_ACCEL, Configs.RoadRunnerConfig.ROTATE_ACCEL))

    override fun isEnd(time: Double) = time > _turn.duration

    override fun transVelocity(time: Double) = Vec2.ZERO

    override fun turnVelocity(time: Double) = _turn[time].velocity().angVel.value()

    override fun targetHeading(time: Double) = Angle(_turn[time].heading.value().toDouble())

    override fun targetPosition(time: Double) = currentPosition
}


open class RunBuildedTrajectory(rawBuildedTrajectory: List<Trajectory>): Action{
    private val _trajectory = Array(rawBuildedTrajectory.size){TimeTrajectory(rawBuildedTrajectory[it])}

    private fun getPoseTime(time: Double): Pose2dDual<Time> {
        var sumDuration = 0.0

        for(i in _trajectory){
            if(i.duration + sumDuration > time)
                return i[time - sumDuration]

            sumDuration += i.duration
        }

        return _trajectory.last()[time]
    }

    fun duration() = _trajectory.sumOf { it.duration }

    override fun isEnd(time: Double) = _trajectory.sumOf { it.duration } < time

    override fun transVelocity(time: Double) = Vec2(getPoseTime(time).velocity().linearVel.value())

    override fun turnVelocity(time: Double) = getPoseTime(time).velocity().angVel.value()

    override fun targetHeading(time: Double) = Angle(getPoseTime(time).heading.value().toDouble())

    override fun targetPosition(time: Double) = Vec2(getPoseTime(time).position.value())
}
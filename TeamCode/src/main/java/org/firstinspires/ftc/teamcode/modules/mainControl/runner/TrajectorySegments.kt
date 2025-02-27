package org.firstinspires.ftc.teamcode.modules.mainControl.runner

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Pose2dDual
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.TimeTurn
import com.acmerobotics.roadrunner.Trajectory
import com.acmerobotics.roadrunner.TurnConstraints
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Orientation
import org.firstinspires.ftc.teamcode.utils.units.Vec2

interface ITrajectorySegment {
    fun isEnd(time: Double): Boolean

    fun transVelocity(time: Double): Vec2

    fun turnVelocity(time: Double): Double

    fun targetOrientation(time: Double): Orientation

    fun duration(): Double
}

class TurnSegment(angle: Double, private val _startOrientation: Orientation): ITrajectorySegment{
    private val _turn = TimeTurn(Pose2d(_startOrientation.x, _startOrientation.y, _startOrientation.angl.angle), angle,
        TurnConstraints(Configs.DriveTrainConfig.MAX_ROTATE_VELOCITY, -Configs.DriveTrainConfig.ROTATE_ACCEL, Configs.DriveTrainConfig.ROTATE_ACCEL))

    override fun isEnd(time: Double) = time > duration()

    override fun transVelocity(time: Double) = Vec2.ZERO

    override fun turnVelocity(time: Double) = _turn[time].velocity().angVel.value()

    override fun targetOrientation(time: Double) = Orientation(_startOrientation.pos, Angle(_turn[time].value().heading.toDouble()))

    override fun duration() = _turn.duration
}

class RRTrajectorySegment(rawBuildedTrajectory: List<Trajectory>): ITrajectorySegment{
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

    override fun isEnd(time: Double) = duration() < time

    override fun transVelocity(time: Double) = Vec2(getPoseTime(time).velocity().linearVel.value())

    override fun turnVelocity(time: Double) = getPoseTime(time).velocity().angVel.value()

    override fun targetOrientation(time: Double) = Orientation(Vec2(getPoseTime(time).position.value()), Angle(getPoseTime(time).heading.value().toDouble()))
    override fun duration() = _trajectory.sumOf { it.duration }
}
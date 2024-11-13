package org.firstinspires.ftc.teamcode.modules.mainControl.runner

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.TimeTurn
import com.acmerobotics.roadrunner.TurnConstraints
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Vec2

interface Action {
    fun isEnd(): Boolean

    fun transVelocity(time: Double): Vec2

    fun turnVelocity(time: Double): Double

    fun turnPosition(time: Double): Double
}

class TurnTo(val angle: Double): Action{
    private val _turn = TimeTurn(Pose2d(0.0, 0.0, 0.0), angle,
        TurnConstraints(Configs.RoadRunnerConfig.MAX_ROTATE_VELOCITY, -Configs.RoadRunnerConfig.ROTATE_ACCEL, Configs.RoadRunnerConfig.ROTATE_ACCEL))

    private val _time = ElapsedTime()

    init {
        _time.reset()
    }

    override fun isEnd(): Boolean = _time.seconds() > _turn.duration

    override fun transVelocity(time: Double) = Vec2.ZERO

    override fun turnVelocity(time: Double) = _turn[_time.seconds()].heading.velocity().value()

    override fun turnPosition(time: Double) = _turn[_time.seconds()].heading.value().toDouble()
}
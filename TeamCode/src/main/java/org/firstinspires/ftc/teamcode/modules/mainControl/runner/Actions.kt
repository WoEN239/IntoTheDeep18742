package org.firstinspires.ftc.teamcode.modules.mainControl.runner

import org.firstinspires.ftc.teamcode.modules.navigation.gyro.Gyro
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.pidRegulator.PIDRegulator
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Vec2
import kotlin.math.abs

interface Action {
    fun isEnd(): Boolean

    fun transVelocity(time: Double): Vec2

    fun turnVelocity(time: Double): Double
}

class TurnTo(val angle: Angle): Action{
    private val _rotPID = PIDRegulator(Configs.RoadRunnerConfig.ROTATED_PID)

    override fun isEnd(): Boolean = abs((Gyro.rotation - angle).angle) < Configs.RoadRunnerConfig.ROTATE_SENS

    override fun transVelocity(time: Double) = Vec2.ZERO

    override fun turnVelocity(time: Double) = _rotPID.update((Gyro.rotation - angle).angle)
}
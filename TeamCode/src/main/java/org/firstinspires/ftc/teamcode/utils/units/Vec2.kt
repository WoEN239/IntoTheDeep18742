package org.firstinspires.ftc.teamcode.utils.units

import com.acmerobotics.roadrunner.Vector2d
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Класс для 2х мерного вектора
 * Переопределы функции сложения, вычитания, умножения, деления
 *
 * @author tikhonsmovzh
 * @see Angle
 * @see Color
 */
data class Vec2(@JvmField var x: Double, @JvmField var y: Double) {
    companion object {
        val ZERO = Vec2(0.0, 0.0)
    }

    constructor(rrVec: Vector2d) : this(rrVec.x, rrVec.y);
    constructor(x: Double) : this(x, x)

    fun length() = sqrt(x * x + y * y)

    fun rot() = atan2(y, x)

    fun setRot(rot: Double): Vec2 {
        val l = length()

        return Vec2(cos(rot) * l, sin(rot) * l)
    }

    fun turn(rot: Double): Vec2 {
        val currentRot = rot()

        return setRot(currentRot + rot)
    }

    operator fun plus(vec: Vec2) = Vec2(x + vec.x, y + vec.y)
    operator fun minus(vec: Vec2) = Vec2(x - vec.x, y - vec.y)
    operator fun times(vec: Vec2) = Vec2(x * vec.x, y * vec.y)
    operator fun div(vec: Vec2) = Vec2(x / vec.x, y / vec.y)

    operator fun times(value: Double) = Vec2(x * value, y * value)
    operator fun div(value: Double) = Vec2(x / value, y / value)
    operator fun plus(value: Double) = Vec2(x + value, y + value)
    operator fun minus(value: Double) = Vec2(x - value, y - value)
}
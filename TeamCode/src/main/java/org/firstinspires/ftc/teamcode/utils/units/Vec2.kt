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
data class Vec2(var x: Double, var y: Double) {
    companion object {
        val ZERO = Vec2(0.0, 0.0)
    }

    constructor(rrVec: Vector2d) : this(rrVec.x, rrVec.y);

    fun length() = x * x + y * y

    fun sqrtLength() = sqrt(length())

    fun setRot(rot: Double): Vec2 {
        val l = sqrtLength()

        return Vec2(cos(rot) * l, sin(rot) * l)
    }

    fun turn(rot: Double): Vec2 {
        val currentRot = atan2(y, x)

        return setRot(currentRot + rot)
    }

    operator fun plus(vec: Vec2) = Vec2(x + vec.x, y + vec.y)
    operator fun minus(vec: Vec2) = Vec2(x - vec.x, y - vec.y)
    operator fun times(vec: Vec2) = Vec2(x * vec.x, y * vec.y)
    operator fun div(vec: Vec2) = Vec2(x / vec.x, y / vec.y)
}
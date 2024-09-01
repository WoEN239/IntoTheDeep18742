package org.firstinspires.ftc.teamcode.utils.units

import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

data class Vec2(var x: Double, var y: Double) {
    val ZERO = Vec2(0.0, 0.0)

    fun length() = x * x + y * y

    fun sqrtLength() = sqrt(length())

    fun setRot(rot: Double) : Vec2 {
        val l = sqrtLength()

        return Vec2(cos(rot) * l, sin(rot) * l)
    }

    operator fun plus(vec: Vec2) = Vec2(x + vec.x, y + vec.y)
    operator fun minus(vec: Vec2) = Vec2(x - vec.x, y - vec.y)
    operator fun times(vec: Vec2) = Vec2(x * vec.x, y * vec.y)
    operator fun div(vec: Vec2) = Vec2(x / vec.x, y / vec.y)
}
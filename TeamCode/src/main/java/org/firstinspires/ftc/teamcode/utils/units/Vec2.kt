package org.firstinspires.ftc.teamcode.utils.units

import kotlin.math.sqrt

class Vec2 (var X: Double, var Y: Double) {
    val ZERO = Vec2(0.0, 0.0)

    fun length() = X * X + Y * Y

    fun sqrtLength() = sqrt(length())
}
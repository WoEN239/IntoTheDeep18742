package org.firstinspires.ftc.teamcode.utils.units

import org.firstinspires.ftc.teamcode.utils.units.Angle.Companion.chop
import java.lang.Math.signum
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sign

/**
 * Класс для угла, все углы в радианах, каждыя оперция проходящая через класс обрезает угол
 * Переопределены функции сложения, вычитания, умножения, деления
 *
 * @see Vec2
 * @see Color
 * @author tikhonsmovzh
 */
data class Angle(var angle: Double){
    companion object {
        fun chop(ang: Double): Double{
            var chopedAng = ang

            while (abs(chopedAng) > PI)
                chopedAng -= 2 * PI * sign(chopedAng)

            return chopedAng
        }
    }

    init {
        angle = chop(angle)
    }

    fun toDegree() = angle / PI * 180

    operator fun plus(ang: Angle) = Angle(chop(angle + ang.angle))
    operator fun plus(ang: Double) = Angle(chop(angle + ang))

    operator fun minus(ang: Angle) = Angle(chop(angle - ang.angle))
    operator fun minus(ang: Double) = Angle(chop(angle - ang))

    operator fun times(ang: Angle) = Angle(chop(angle * ang.angle))
    operator fun times(ang: Double) = Angle(chop(angle * ang))

    operator fun div(ang: Angle) = Angle(chop(angle / ang.angle))
    operator fun div(ang: Double) = Angle(chop(angle / ang))
}

operator fun Double.plus(ang: Angle) = Angle(chop(this + ang.angle))
operator fun Double.minus(ang: Angle) = Angle(chop(this - ang.angle))
operator fun Double.times(ang: Angle) = Angle(chop(this * ang.angle))
operator fun Double.div(ang: Angle) = Angle(chop(this / ang.angle))
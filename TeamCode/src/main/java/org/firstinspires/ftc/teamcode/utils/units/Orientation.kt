package org.firstinspires.ftc.teamcode.utils.units

data class Orientation(var pos: Vec2, var angl: Angle) {
    companion object{
        val ZERO = Orientation(0.0)
    }

    var x
        get() = pos.x
        set(value){
            pos.x = value
        }

    var y
        get() = pos.y
        set(value) {
            pos.y = value
        }

    constructor(x: Double): this(Vec2(x), Angle(x))
    constructor(x: Vec2): this(x, Angle.ZERO)
    constructor(x: Angle): this(Vec2.ZERO, x)
    constructor(): this(Vec2.ZERO, Angle.ZERO)

    operator fun plus(orientation: Orientation) = Orientation(pos + orientation.pos, angl + orientation.angl)
    operator fun minus(orientation: Orientation) = Orientation(pos - orientation.pos, angl - orientation.angl)
    operator fun times(orientation: Orientation) = Orientation(pos * orientation.pos, angl * orientation.angl)
    operator fun div(orientation: Orientation) = Orientation(pos / orientation.pos, angl / orientation.angl)
}
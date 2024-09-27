package org.firstinspires.ftc.teamcode.modules.navigation.odometry

import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.utils.units.Vec2

object MergeOdometry: IRobotModule {
    override fun init(collector: BaseCollector) {

    }

    var position = Vec2.ZERO
    var velocity = Vec2.ZERO

    override fun lateUpdate() {
        position = OdometersOdometry.position
        velocity = OdometersOdometry.velocity
    }
}
package org.firstinspires.ftc.teamcode.modules.navigation.odometry

import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.modules.navigation.gyro.MergeGyro
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Vec2

object OdometersOdometry : IRobotModule {
    override fun init(collector: BaseCollector) {}

    var position = Vec2.ZERO
    var velocity = Vec2.ZERO

    private var _oldRotate = Angle(0.0)

    override fun update() {
        val deltaForwardOdometerLeft = HardwareOdometers.forwardOdometerLeftPosition - HardwareOdometers.oldPositionForwardOdometerLeft
        val deltaForwardOdometerRight = HardwareOdometers.forwardOdometerRightPosition - HardwareOdometers.oldPositionForwardOdometerRight
        val deltaSideOdometer = HardwareOdometers.sideOdometerPosition - HardwareOdometers.oldPositionSideOdometer
        val deltaRotate = MergeGyro.rotation - _oldRotate

        position += Vec2(
            (deltaForwardOdometerRight + deltaForwardOdometerLeft) / 2.0,
            deltaSideOdometer - (Angle(Configs.OdometryConfig.SIDE_ODOMETER_RADIUS) * deltaRotate).angle
        ).turn(MergeGyro.rotation.angle)

        velocity = Vec2(
            (HardwareOdometers.forwardOdometerLeftVelocity + HardwareOdometers.forwardOdometerRightVelocity) / 2.0,
            HardwareOdometers.sideOdometerVelocity - Configs.OdometryConfig.SIDE_ODOMETER_RADIUS * MergeGyro.velocity
        )
        _oldRotate = MergeGyro.rotation
    }
}
package org.firstinspires.ftc.teamcode.modules.navigation.gyro

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.units.Angle

object IMUGyro: IRobotModule {
    private lateinit var _imu: IMU
    private val _oldReadTime = ElapsedTime()

    override fun init(collector: BaseCollector, bus: EventBus) {
        _imu = collector.devices.imu

        _imu.initialize(
            IMU.Parameters(RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP)))
    }

    override fun start() {
        _imu.resetYaw()
    }

    private var _oldRot = Angle(0.0)

    fun calculateRotate(): Angle{
        if(_oldReadTime.milliseconds() > 1000.0 / Configs.GyroscopeConfig.READ_HZ) {
            _oldRot = Angle(_imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS))

            _oldReadTime.reset()
        }

        return _oldRot
    }
}
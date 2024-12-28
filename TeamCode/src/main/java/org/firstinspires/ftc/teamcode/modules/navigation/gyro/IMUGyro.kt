package org.firstinspires.ftc.teamcode.modules.navigation.gyro

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.units.Angle

class IMUGyro: IRobotModule {
    private lateinit var _imu: IMU
    private val _oldReadTime = ElapsedTime()

    private lateinit var _eventBus: EventBus

    private var _startAngle = Angle.ZERO

    override fun init(collector: BaseCollector, bus: EventBus) {
        _startAngle = collector.gameSettings.startPosition.angle

        _imu = collector.devices.imu

        _imu.initialize(
            IMU.Parameters(RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)))

        _eventBus = bus
    }

    override fun start() {
        _imu.resetYaw()
    }

    private var _oldRot = Angle(0.0)

    override fun update() {
        if(_oldReadTime.milliseconds() > 1000.0 / Configs.GyroscopeConfig.READ_HZ) {
            val raw = _imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)

            _oldRot = Angle(raw) + _startAngle

            _oldReadTime.reset()

            _eventBus.invoke(UpdateImuGyroEvent(_oldRot))
        }
    }

    class UpdateImuGyroEvent(val rotate: Angle): IEvent

}
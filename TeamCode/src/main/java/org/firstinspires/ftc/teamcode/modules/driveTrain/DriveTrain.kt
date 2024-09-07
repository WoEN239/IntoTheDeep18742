package org.firstinspires.ftc.teamcode.modules.driveTrain

import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.motor.Motor
import org.firstinspires.ftc.teamcode.utils.units.Vec2

object DriveTrain : IRobotModule {
    private lateinit var _leftForwardDrive: Motor
    private lateinit var _rightForwardDrive: Motor
    private lateinit var _leftBackDrive: Motor
    private lateinit var _rightBackDrive: Motor

    override fun init(collector: BaseCollector) {
        _leftForwardDrive = Motor(collector.devices.leftForwardDrive)
        _rightForwardDrive = Motor(collector.devices.rightForwardDrive)
        _leftBackDrive = Motor(collector.devices.leftBackDrive)
        _rightBackDrive = Motor(collector.devices.rightBackDrive)
    }

    fun driveSimpleDirection(direction: Vec2, rotate: Double) {
        _leftForwardDrive.targetPower = direction.x - direction.y - rotate
        _rightBackDrive.targetPower = direction.x - direction.y + rotate
        _leftBackDrive.targetPower = direction.x + direction.y - rotate
        _rightForwardDrive.targetPower = direction.x + direction.y + rotate
    }

    fun driveTicksDirection(direction: Vec2, rotate: Double) {
        _leftForwardDrive.targetTicksVelocity = direction.x - direction.y - rotate
        _rightBackDrive.targetTicksVelocity = direction.x - direction.y + rotate
        _leftBackDrive.targetTicksVelocity = direction.x + direction.y - rotate
        _rightForwardDrive.targetTicksVelocity = direction.x + direction.y + rotate
    }

    fun driveCmDirection(direction: Vec2, rotate: Double) = driveTicksDirection(
        Vec2(
            direction.x / Configs.DriveTrainConfig.WHEEL_DIAMETER * Configs.DriveTrainConfig.WHEEL_ENCODER_CONSTANT,
            direction.y / Configs.DriveTrainConfig.WHEEL_DIAMETER * Configs.DriveTrainConfig.WHEEL_ENCODER_CONSTANT * Configs.DriveTrainConfig.Y_LAG
        ),
        rotate * Configs.DriveTrainConfig.WHEEL_CENTER_RADIUS / Configs.DriveTrainConfig.WHEEL_DIAMETER * Configs.DriveTrainConfig.WHEEL_ENCODER_CONSTANT
    )

    override fun stop(){
        driveSimpleDirection(Vec2(0.0, 0.0), 0.0)
    }
}
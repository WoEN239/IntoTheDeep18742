package org.firstinspires.ftc.teamcode.modules.driveTrain

import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.utils.units.Vec2

object DriveTrain : IRobotModule {
    private lateinit var _leftForwardDrive: DcMotorEx
    private lateinit var _rightForwardDrive: DcMotorEx
    private lateinit var _leftBackDrive: DcMotorEx
    private lateinit var _rightBackDrive: DcMotorEx

    override fun init(collector: BaseCollector) {
        _leftForwardDrive = collector.devices.leftForwardDrive
        _rightForwardDrive = collector.devices.rightForwardDrive
        _leftBackDrive = collector.devices.leftBackDrive
        _rightBackDrive = collector.devices.rightBackDrive
    }

    fun driveDirection(direction: Vec2, rotate: Double) {
        _leftForwardDrive.power = direction.x - direction.y - rotate;
        _rightBackDrive.power = direction.x - direction.y + rotate;
        _leftBackDrive.power = direction.x + direction.y - rotate;
        _rightForwardDrive.power = direction.x + direction.y + rotate;
    }
}
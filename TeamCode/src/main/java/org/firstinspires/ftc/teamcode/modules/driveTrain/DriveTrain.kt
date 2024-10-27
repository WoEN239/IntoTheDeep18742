package org.firstinspires.ftc.teamcode.modules.driveTrain

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.pidRegulator.PIDRegulator
import org.firstinspires.ftc.teamcode.utils.units.Vec2

object DriveTrain : IRobotModule {
    private lateinit var _leftForwardDrive: DcMotorEx
    private lateinit var _rightForwardDrive: DcMotorEx
    private lateinit var _leftBackDrive: DcMotorEx
    private lateinit var _rightBackDrive: DcMotorEx

    private val _velocityPidfForward = PIDRegulator(Configs.DriveTrainConfig.VELOCITY_PIDF_FORWARD)
    private val _velocityPidfSide = PIDRegulator(Configs.DriveTrainConfig.VELOCITY_PIDF_SIDE)
    private val _velocityPidfRotate = PIDRegulator(Configs.DriveTrainConfig.VELOCITY_PIDF_ROTATE)

    private var _isAuto: Boolean = false

    override fun init(collector: BaseCollector) {
        _isAuto = collector.gameSettings.isAuto

        _leftForwardDrive = collector.devices.leftForwardDrive
        _rightForwardDrive = collector.devices.rightForwardDrive
        _leftBackDrive = collector.devices.leftBackDrive
        _rightBackDrive = collector.devices.rightBackDrive

        _rightBackDrive.direction = DcMotorSimple.Direction.REVERSE
        _rightForwardDrive.direction = DcMotorSimple.Direction.REVERSE
    }

    private fun driveSimpleDirection(direction: Vec2, rotate: Double) {
        _leftForwardDrive.power = direction.x - direction.y - rotate
        _rightBackDrive.power = direction.x - direction.y + rotate
        _leftBackDrive.power = direction.x + direction.y - rotate
        _rightForwardDrive.power = direction.x + direction.y + rotate
    }

    private var _targetDirectionVelocity = Vec2.ZERO
    private var _targetRotateVelocity = 0.0

    fun driveTicksDirection(direction: Vec2, rotate: Double) {
        _targetDirectionVelocity = direction
        _targetRotateVelocity = rotate
    }

    fun driveCmDirection(direction: Vec2, rotate: Double) = driveTicksDirection(
        Vec2(
            direction.x / Configs.OdometryConfig.ODOMETER_DIAMETER * Configs.OdometryConfig.ODOMETER_TICKS,
            direction.y / Configs.OdometryConfig.ODOMETER_DIAMETER * Configs.OdometryConfig.ODOMETER_TICKS * Configs.DriveTrainConfig.Y_LAG
        ),
        rotate * Configs.DriveTrainConfig.WHEEL_CENTER_RADIUS / Configs.OdometryConfig.ODOMETER_DIAMETER * Configs.OdometryConfig.ODOMETER_TICKS
    )


    fun drivePowerDirection(direction: Vec2, rotate: Double) =/* driveTicksDirection(
        direction * Vec2(Configs.DriveTrainConfig.MAX_SPEED_FORWARD, Configs.DriveTrainConfig.MAX_SPEED_SIDE), rotate * Configs.DriveTrainConfig.MAX_SPEED_TURN)*/
        driveSimpleDirection(
            if (!_isAuto && Intake.position == Intake.AdvancedPosition.SERVO_PROMOTED) direction * Vec2(
                Configs.DriveTrainConfig.VELOSITY_SLOW_K,
                Configs.DriveTrainConfig.VELOSITY_SLOW_K
            ) else direction * Vec2(1.0, Configs.DriveTrainConfig.Y_LAG), rotate
        )

    override fun stop() {
        drivePowerDirection(Vec2(0.0, 0.0), 0.0)
    }

    override fun update() {
        /*driveSimpleDirection(Vec2(
            _velocityPidfForward.update(_targetDirectionVelocity.x - MergeOdometry.velocity.x, _targetDirectionVelocity.x),
            _velocityPidfSide.update(_targetDirectionVelocity.y - MergeOdometry.velocity.y, _targetDirectionVelocity.y)),
            _velocityPidfRotate.update(_targetRotateVelocity - MergeGyro.velocity, MergeGyro.velocity))*/
    }
}
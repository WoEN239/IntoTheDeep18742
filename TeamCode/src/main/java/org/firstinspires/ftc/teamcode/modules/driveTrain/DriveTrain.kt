package org.firstinspires.ftc.teamcode.modules.driveTrain

import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.modules.intake.IntakeManager
import org.firstinspires.ftc.teamcode.modules.navigation.gyro.MergeGyro
import org.firstinspires.ftc.teamcode.modules.navigation.odometry.MergeOdometry
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.devices.Battery
import org.firstinspires.ftc.teamcode.utils.pidRegulator.PIDRegulator
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.units.Vec2
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

class DriveTrain : IRobotModule {
    private lateinit var _leftForwardDrive: DcMotorEx
    private lateinit var _rightForwardDrive: DcMotorEx
    private lateinit var _leftBackDrive: DcMotorEx
    private lateinit var _rightBackDrive: DcMotorEx

    private val _velocityPidfForward = PIDRegulator(Configs.DriveTrainConfig.VELOCITY_PIDF_FORWARD)
    private val _velocityPidfSide = PIDRegulator(Configs.DriveTrainConfig.VELOCITY_PIDF_SIDE)
    private val _velocityPidfRotate = PIDRegulator(Configs.DriveTrainConfig.VELOCITY_PIDF_ROTATE)

    private var _isAuto: Boolean = false

    private lateinit var _eventBus: EventBus

    private lateinit var _battery: Battery

    override fun init(collector: BaseCollector, bus: EventBus) {
        _eventBus = bus

        _battery = collector.devices.battery
        _isAuto = collector.gameSettings.isAuto

        _leftForwardDrive = collector.devices.leftForwardDrive
        _rightForwardDrive = collector.devices.rightForwardDrive
        _leftBackDrive = collector.devices.leftBackDrive
        _rightBackDrive = collector.devices.rightBackDrive

        _leftForwardDrive.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        _rightForwardDrive.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        _leftBackDrive.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        _rightBackDrive.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        _leftBackDrive.direction = DcMotorSimple.Direction.REVERSE
        _leftForwardDrive.direction = DcMotorSimple.Direction.REVERSE

        bus.subscribe(SetDrivePowerEvent::class){
            var dir = it.direction
            var rot = it.rotate

            if(_eventBus.invoke(IntakeManager.RequestLiftPosEvent()).pos != IntakeManager.LiftPosition.TRANSPORT) {
                dir *= Vec2(Configs.DriveTrainConfig.LIFT_MAX_SPEED)
                rot *= Configs.DriveTrainConfig.LIFT_MAX_SPEED
            }

            bus.invoke(SetDriveCmEvent(dir * Vec2(Configs.DriveTrainConfig.MAX_TRANSLATION_VELOCITY, Configs.DriveTrainConfig.MAX_ROTATE_VELOCITY), rot * Configs.DriveTrainConfig.MAX_ROTATE_VELOCITY))
        }

        bus.subscribe(SetDriveCmEvent::class){
            var clampedDirLength = clamp(it.direction.length(), -Configs.DriveTrainConfig.MAX_TRANSLATION_VELOCITY, Configs.DriveTrainConfig.MAX_TRANSLATION_VELOCITY)
            val dirRot = it.direction.rot()

            _targetDirectionVelocity = Vec2(cos(dirRot) * clampedDirLength, sin(dirRot) * clampedDirLength)
            _targetRotateVelocity = clamp(it.rotate, -Configs.DriveTrainConfig.MAX_ROTATE_VELOCITY, Configs.DriveTrainConfig.MAX_ROTATE_VELOCITY)
        }

        bus.subscribe(MergeOdometry.UpdateMergeOdometryEvent::class){
            val gyro = bus.invoke(MergeGyro.RequestMergeGyroEvent())

            driveSimpleDirection(Vec2(
                _velocityPidfForward.update(_targetDirectionVelocity.x - it.velocity.x, _targetDirectionVelocity.x),
                _velocityPidfSide.update(_targetDirectionVelocity.y - it.velocity.y, _targetDirectionVelocity.y)),
                _velocityPidfRotate.update(_targetRotateVelocity - gyro.velocity!!, _targetRotateVelocity))
        }
    }

    private fun driveSimpleDirection(direction: Vec2, rotate: Double) {
        var leftFrontPower = _battery.voltageToPower(direction.x - direction.y + rotate)
        var rightBackPower = _battery.voltageToPower(direction.x - direction.y - rotate)
        var leftBackPower = _battery.voltageToPower(direction.x + direction.y + rotate)
        var rightForwardPower = _battery.voltageToPower(direction.x + direction.y - rotate)

        val max = max(abs(leftFrontPower), max(abs(rightBackPower), max(abs(leftBackPower), abs(rightForwardPower))))

        if(max > 1.0){
            leftFrontPower /= max
            rightBackPower /= max
            leftBackPower /= max
            rightForwardPower /= max
        }

        _leftForwardDrive.power = leftFrontPower
        _rightBackDrive.power = rightBackPower
        _leftBackDrive.power = leftBackPower
        _rightForwardDrive.power = rightForwardPower
    }

    private var _targetDirectionVelocity = Vec2.ZERO
    private var _targetRotateVelocity = 0.0

    override fun stop() {
        _targetDirectionVelocity = Vec2.ZERO
        _targetRotateVelocity = 0.0
    }

    class SetDrivePowerEvent(val direction: Vec2, val rotate: Double): IEvent
    class SetDriveCmEvent(val direction: Vec2, val rotate: Double): IEvent
}
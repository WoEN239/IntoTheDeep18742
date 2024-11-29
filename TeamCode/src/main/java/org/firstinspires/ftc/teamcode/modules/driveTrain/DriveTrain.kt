package org.firstinspires.ftc.teamcode.modules.driveTrain

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.modules.navigation.gyro.MergeGyro
import org.firstinspires.ftc.teamcode.modules.navigation.odometry.MergeOdometry
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.pidRegulator.PIDRegulator
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.timer.Timers
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Vec2

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

    override fun init(collector: BaseCollector, bus: EventBus) {
        _eventBus = bus

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
            bus.invoke(SetDriveCmEvent(it.direction * Vec2(Configs.DriveTrainConfig.MAX_SPEED_FORWARD, Configs.DriveTrainConfig.MAX_SPEED_SIDE), it.rotate * Configs.DriveTrainConfig.MAX_SPEED_TURN))
        }

        bus.subscribe(SetDriveCmEvent::class){
            _targetDirectionVelocity = it.direction
            _targetRotateVelocity = it.rotate
        }

        bus.subscribe(MergeOdometry.UpdateMergeOdometryEvent::class){
            val gyro = bus.invoke(MergeGyro.RequestMergeGyroEvent())

            driveSimpleDirection(Vec2(
                _velocityPidfForward.update(_targetDirectionVelocity.x - it.velocity.x, _targetDirectionVelocity.x) / collector.devices.battery.charge,
                _velocityPidfSide.update(_targetDirectionVelocity.y - it.velocity.y, _targetDirectionVelocity.y) / collector.devices.battery.charge),
                _velocityPidfRotate.update(_targetRotateVelocity - gyro.velocity!!, _targetRotateVelocity) / collector.devices.battery.charge)
        }

        bus.subscribe(SetLocalDriveCm::class){
            val gyro = bus.invoke(MergeGyro.RequestMergeGyroEvent())

            bus.invoke(SetDriveCmEvent(it.direction.turn(gyro.rotation!!.angle), it.rotate))
        }
    }

    private fun driveSimpleDirection(direction: Vec2, rotate: Double) {
        _leftForwardDrive.power = -direction.x - direction.y + rotate
        _rightBackDrive.power = -direction.x - direction.y - rotate
        _leftBackDrive.power = -direction.x + direction.y + rotate
        _rightForwardDrive.power = -direction.x + direction.y - rotate
    }

    private var _targetDirectionVelocity = Vec2.ZERO
    private var _targetRotateVelocity = 0.0

    override fun stop() {
        _targetDirectionVelocity = Vec2.ZERO
        _targetRotateVelocity = 0.0
    }

    override fun start() {
        /*var action = {}

        action = {
            _eventBus.invoke(SetDriveCmEvent(Vec2(0.0, -50.0), 0.0))

            Timers.newTimer().start(1.8){
                _eventBus.invoke(SetDriveCmEvent(Vec2(0.0, 50.0), 0.0))

                Timers.newTimer().start(1.8, action)
            }
        }

        action.invoke()*/
    }

    class SetDrivePowerEvent(val direction: Vec2, val rotate: Double): IEvent
    class SetDriveCmEvent(val direction: Vec2, val rotate: Double): IEvent
    class SetLocalDriveCm(val direction: Vec2, val rotate: Double): IEvent
}
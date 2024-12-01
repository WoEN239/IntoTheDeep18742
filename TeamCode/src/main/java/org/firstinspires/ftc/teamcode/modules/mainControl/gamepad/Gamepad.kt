package org.firstinspires.ftc.teamcode.modules.mainControl.gamepad

import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.modules.driveTrain.DriveTrain
import org.firstinspires.ftc.teamcode.modules.driveTrain.DriveTrain.SetDrivePowerEvent
import org.firstinspires.ftc.teamcode.modules.hook.Hook
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.modules.intake.Intake.ClampPosition
import org.firstinspires.ftc.teamcode.modules.intake.Intake.SetClampPoseEvent
import org.firstinspires.ftc.teamcode.modules.intake.Intake.SetDifDownEvent
import org.firstinspires.ftc.teamcode.modules.intake.Intake.SetDifUpEvent
import org.firstinspires.ftc.teamcode.modules.lift.Lift
import org.firstinspires.ftc.teamcode.modules.lift.Lift.SetLiftTargetEvent
import org.firstinspires.ftc.teamcode.modules.navigation.gyro.MergeGyro
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.units.Vec2

class Gamepad : IRobotModule {
    private lateinit var _gamepad: Gamepad
    private lateinit var _eventBus: EventBus

    override fun init(collector: BaseCollector, bus: EventBus) {
        _gamepad = collector.robot.gamepad1
        _eventBus = bus
    }

    override fun start() {
        _deltaTime.reset()
    }

    private val _deltaTime = ElapsedTime()

    private var _oldClamp = false
    private var _clampPos = false

    private var _oldDif = false
    private var _difPos = false

    override fun lateUpdate() {
        _eventBus.invoke(
            SetDrivePowerEvent(
                Vec2(
                    (-_gamepad.left_stick_y).toDouble(),
                    (_gamepad.left_stick_x).toDouble()
                ), (-_gamepad.right_stick_x).toDouble()
            )
        )

        val liftTarget = _eventBus.invoke(Lift.RequestCurrentLiftTarget())

        _eventBus.invoke(
            SetLiftTargetEvent(
                liftTarget.aimPos +
                        _deltaTime.seconds() * (if (_gamepad.dpad_up) Configs.LiftConfig.AIM_GAMEPAD_SENS else 0.0) -
                        _deltaTime.seconds() * (if (_gamepad.dpad_down) Configs.LiftConfig.AIM_GAMEPAD_SENS else 0.0),
                liftTarget.extensionPos +
                        _deltaTime.seconds() * (if (_gamepad.dpad_right) Configs.LiftConfig.EXTENSION_GAMEPAD_SENS else 0.0) -
                        _deltaTime.seconds() * (if (_gamepad.dpad_left) Configs.LiftConfig.EXTENSION_GAMEPAD_SENS else 0.0)
            )
        )

        _deltaTime.reset()

        if(_gamepad.circle && !_oldClamp)
            _clampPos = !_clampPos

        _oldClamp = _gamepad.circle

        _eventBus.invoke(SetClampPoseEvent(if(_clampPos)  ClampPosition.SERVO_CLAMP else  ClampPosition.SERVO_UNCLAMP))

        if(_gamepad.triangle)
            _eventBus.invoke(Hook.HookRun())
        else
            _eventBus.invoke(Hook.HookStop())

        if(!_oldDif && _gamepad.square)
            _difPos = !_difPos

        _oldDif = _gamepad.square

        if(_difPos)
            _eventBus.invoke(SetDifUpEvent())
        else
            _eventBus.invoke(SetDifDownEvent())
    }
}
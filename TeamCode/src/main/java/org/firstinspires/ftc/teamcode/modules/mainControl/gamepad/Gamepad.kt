package org.firstinspires.ftc.teamcode.modules.mainControl.gamepad

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.modules.driveTrain.DriveTrain.SetDrivePowerEvent
import org.firstinspires.ftc.teamcode.modules.hook.Hook
import org.firstinspires.ftc.teamcode.modules.intake.Intake.ClampPosition
import org.firstinspires.ftc.teamcode.modules.intake.Intake.SetClampPoseEvent
import org.firstinspires.ftc.teamcode.utils.units.Vec2

class Gamepad : IRobotModule {
    private lateinit var _gamepad: Gamepad
    private lateinit var _eventBus: EventBus

    override fun init(collector: BaseCollector, bus: EventBus) {
        _gamepad = collector.robot.gamepad1
        _eventBus = bus
    }

    private var _oldClamp = false
    private var _clampPos = false

    override fun lateUpdate() {
        _eventBus.invoke(
            SetDrivePowerEvent(
                Vec2(
                    (-_gamepad.left_stick_y).toDouble(),
                    (_gamepad.left_stick_x).toDouble()
                ), (-_gamepad.right_stick_x).toDouble()
            )
        )

        if(_gamepad.circle && !_oldClamp)
            _clampPos = !_clampPos

        _oldClamp = _gamepad.circle

        _eventBus.invoke(SetClampPoseEvent(if(_clampPos)  ClampPosition.SERVO_CLAMP else  ClampPosition.SERVO_UNCLAMP))

        if(_gamepad.triangle)
            _eventBus.invoke(Hook.HookRun())
        else
            _eventBus.invoke(Hook.HookStop())
    }
}
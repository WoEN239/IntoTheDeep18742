package org.firstinspires.ftc.teamcode.modules.mainControl.gamepad

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.modules.driveTrain.DriveTrain.SetDrivePowerEvent
import org.firstinspires.ftc.teamcode.modules.hook.Hook
import org.firstinspires.ftc.teamcode.modules.intake.Intake.ClampPosition
import org.firstinspires.ftc.teamcode.modules.intake.IntakeManager
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.units.Vec2

class Gamepad : IRobotModule {
    private lateinit var _gamepad: Gamepad
    private lateinit var _eventBus: EventBus

    override fun init(collector: BaseCollector, bus: EventBus) {
        _gamepad = collector.robot.gamepad1
        _eventBus = bus
    }

    private var _oldClamp = false

    private var _basketOld = false
    private var _centerOld = false
    private var _layerOld = false
    private var _clampWallOld = false

    private var _oldNextDifPos = false
    private var _oldPreviousDifPos = false

    override fun lateUpdate() {
        _eventBus.invoke(
            SetDrivePowerEvent(
                Vec2(
                    (-_gamepad.left_stick_y).toDouble(),
                    (-_gamepad.left_stick_x).toDouble()
                ), (-_gamepad.right_stick_x).toDouble()
            )
        )

        if (_gamepad.circle && !_oldClamp) {
            if (_eventBus.invoke(IntakeManager.RequestClampPosEvent()).pos == ClampPosition.SERVO_UNCLAMP)
                _eventBus.invoke(IntakeManager.EventSetClampPose(ClampPosition.SERVO_CLAMP))
            else
                _eventBus.invoke(IntakeManager.EventSetClampPose(ClampPosition.SERVO_UNCLAMP))
        }

        _oldClamp = _gamepad.circle

        if (_gamepad.triangle)
            _eventBus.invoke(Hook.HookRun())
        else if (_gamepad.square)
            _eventBus.invoke(Hook.HookRunRevers())
        else
            _eventBus.invoke(Hook.HookStop())

        if (!_basketOld && _gamepad.dpad_up)
            _eventBus.invoke(IntakeManager.EventSetLiftPose(IntakeManager.LiftPosition.UP_BASKED))

        if (!_centerOld && _gamepad.dpad_down)
            _eventBus.invoke(IntakeManager.EventSetLiftPose(IntakeManager.LiftPosition.CLAMP_CENTER))

        if (!_layerOld && _gamepad.dpad_right)
            _eventBus.invoke(IntakeManager.EventSetLiftPose(IntakeManager.LiftPosition.UP_LAYER))

        if (!_clampWallOld && _gamepad.dpad_left)
            _eventBus.invoke(IntakeManager.EventSetLiftPose(IntakeManager.LiftPosition.CLAMP_WALL))

        _basketOld = _gamepad.dpad_up
        _centerOld = _gamepad.dpad_down
        _layerOld = _gamepad.dpad_right
        _clampWallOld = _gamepad.dpad_left

        _eventBus.invoke(IntakeManager.EventSetExtensionVel((_gamepad.right_trigger - _gamepad.left_trigger).toDouble() * Configs.LiftConfig.GAMEPAD_EXTENSION_SENS))

        if(_gamepad.right_bumper && !_oldNextDifPos)
            _eventBus.invoke(IntakeManager.NextDifPos())

        if(_gamepad.left_bumper && !_oldPreviousDifPos)
            _eventBus.invoke(IntakeManager.PreviousDifPos())

        _oldNextDifPos = _gamepad.right_bumper
        _oldPreviousDifPos = _gamepad.left_bumper
    }
}
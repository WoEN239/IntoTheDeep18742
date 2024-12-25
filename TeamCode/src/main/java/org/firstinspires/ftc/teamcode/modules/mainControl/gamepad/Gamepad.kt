package org.firstinspires.ftc.teamcode.modules.mainControl.gamepad

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.modules.driveTrain.DriveTrain.SetDrivePowerEvent
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

    private var _basketOld = false
    private var _centerOld = false
    private var _wallOld = false
    private var _layerOld = false

    override fun lateUpdate() {
        _eventBus.invoke(
            SetDrivePowerEvent(
                Vec2(
                    (-_gamepad.left_stick_y).toDouble(),
                    (_gamepad.left_stick_x).toDouble()
                ), (-_gamepad.right_stick_x).toDouble()
            )
        )

        /*if(_gamepad.circle && !_oldClamp) {
            _clampPos = !_clampPos

            if(_clampPos)
                _eventBus.invoke(SetClampStateEvent(ClampPosition.SERVO_CLAMP))
            else
                _eventBus.invoke(SetClampStateEvent(ClampPosition.SERVO_UNCLAMP))
        }

        _oldClamp = _gamepad.circle

        if(_gamepad.triangle)
            _eventBus.invoke(Hook.HookRun())
        else if(_gamepad.square)
            _eventBus.invoke(Hook.HookRunRevers())
        else
            _eventBus.invoke(Hook.HookStop())

        if(!_basketOld && _gamepad.dpad_up)
            _eventBus.invoke(SetLiftStateEvent(Lift.LiftStates.UP_BASKET))

        if(!_centerOld && _gamepad.dpad_down)
            _eventBus.invoke(SetLiftStateEvent(Lift.LiftStates.CLAMP_CENTER))

        /*if(!_layerOld && _gamepad.dpad_right)
            _eventBus.invoke(SetLiftStateEvent(Lift.LiftStates.UP_LAYER))

        if(!_wallOld &&_gamepad.dpad_left)
            _eventBus.invoke(SetLiftStateEvent(Lift.LiftStates.CLAMP_WALL))*/

        _basketOld = _gamepad.dpad_up
        _centerOld = _gamepad.dpad_down
        _layerOld = _gamepad.dpad_right
        _wallOld = _gamepad.dpad_left

        _eventBus.invoke(Lift.SetExtensionVelocityEvent((_gamepad.right_trigger - _gamepad.left_trigger).toDouble() * Configs.LiftConfig.GAMEPAD_EXTENSION_SENS))

        _eventBus.invoke(Intake.SetDifVelocityEvent(0.0, if(_gamepad.right_bumper) Configs.IntakeConfig.DIX_Y_VELOCITY else if(_gamepad.left_bumper) -Configs.IntakeConfig.DIX_Y_VELOCITY else 0.0))*/
    }
}
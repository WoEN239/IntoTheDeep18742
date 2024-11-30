package org.firstinspires.ftc.teamcode.modules.mainControl.gamepad

import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.modules.driveTrain.DriveTrain
import org.firstinspires.ftc.teamcode.modules.driveTrain.DriveTrain.SetDrivePowerEvent
import org.firstinspires.ftc.teamcode.modules.intake.Intake
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

    private var _promotedOld = false
    private var _clampOld = false
    private var _servoflip = false
    private var _clampOldF = false
    private var _clampOldU = false
    private var _rotateOldU = false

    private val _deltaTime = ElapsedTime()

    private var _liftPoseAim = 0.0
    private var _liftPosePromoted = 0.0

    override fun lateUpdate() {
        _eventBus.invoke(SetDrivePowerEvent(Vec2((-_gamepad.left_stick_y).toDouble(), (_gamepad.left_stick_x).toDouble()), (_gamepad.right_stick_x).toDouble()))

        _liftPoseAim += _deltaTime.seconds() *  (if(_gamepad.dpad_up) 300.0 else 0.0)
        _liftPoseAim -= _deltaTime.seconds() *  (if(_gamepad.dpad_down) 300.0 else 0.0)

        _liftPosePromoted += _deltaTime.seconds() *  (if(_gamepad.dpad_right) 200.0 else 0.0)
        _liftPosePromoted -= _deltaTime.seconds() *  (if(_gamepad.dpad_left) 200.0 else 0.0)

        _liftPoseAim = clamp(_liftPoseAim, 0.0, 680.0)
        _liftPosePromoted = clamp(_liftPosePromoted, 0.0, 2275.0)

        _eventBus.invoke(SetLiftTargetEvent(_liftPoseAim, _liftPosePromoted))

        _deltaTime.reset()
        /*DriveTrain.drivePowerDirection(
            Vec2(
                (_gamepad.left_stick_y).toDouble(),
                (_gamepad.left_stick_x).toDouble()
            ).turn(-MergeGyro.rotation.angle),
            (_gamepad.right_stick_x).toDouble()
        )

        if (_gamepad.cross)
            Lift.targetPosition = Lift.LiftPosition.MIDDLE
        if (_gamepad.triangle)
            Lift.targetPosition = Lift.LiftPosition.UP
        if (_gamepad.circle)
            Lift.targetPosition = Lift.LiftPosition.DOWN

   /*     if (_gamepad.dpad_down && !_promotedOld) {
            if (Intake.position == Intake.AdvancedPosition.SERVO_UNPROMOTED){
                Intake.position = Intake.AdvancedPosition.SERVO_PROMOTED
            Intake.flip = Intake.GalaxyFlipPosition.SERVO_FLIP

            }
            else{
                Intake.position = Intake.AdvancedPosition.SERVO_UNPROMOTED
            Intake.flip = Intake.GalaxyFlipPosition.SERVO_UNFLIP}
        }

        _promotedOld = _gamepad.dpad_down
*/
        if (_gamepad.dpad_up && !_clampOld)
            if (Intake.clamp == Intake.ClampPosition.SERVO_UNCLAMP){
                Intake.clamp = Intake.ClampPosition.SERVO_CLAMP
                Intake.clampF = Intake.ClampPositionF.SERVO_CLAMPF
                Intake.position = Intake.AdvancedPosition.SERVO_UNPROMOTED
                Intake.flip = Intake.GalaxyFlipPosition.SERVO_UNFLIP
            }
            else{
                Intake.clampF = Intake.ClampPositionF.SERVO_UNCLAMPF
                Intake.position = Intake.AdvancedPosition.SERVO_PROMOTED
                Intake.flip = Intake.GalaxyFlipPosition.SERVO_FLIP
                Intake.clamp = Intake.ClampPosition.SERVO_UNCLAMP
            }

        _clampOld = _gamepad.dpad_up

        if (_gamepad.dpad_right && !_clampOldU)
            if (Intake.clampUp == Intake.ClampPositionUp.SERVO_UNCLAMPUP)
                Intake.clampUp = Intake.ClampPositionUp.SERVO_CLAMPUP
            else
                Intake.clampUp = Intake.ClampPositionUp.SERVO_UNCLAMPUP

        _clampOldU = _gamepad.dpad_right

        if (_gamepad.dpad_left && !_rotateOldU)
            if (Intake.rotateUp == Intake.RotatePositionUp.SERVO_UNROTATEUP)
                Intake.rotateUp = Intake.RotatePositionUp.SERVO_ROTATEUP
            else
                Intake.rotateUp = Intake.RotatePositionUp.SERVO_UNROTATEUP

        _rotateOldU = _gamepad.dpad_left
            /*   if (_gamepad.dpad_right && !_servoflip)
            if (Intake.flip == Intake.GalaxyFlipPosition.SERVO_UNFLIP)
                Intake.flip = Intake.GalaxyFlipPosition.SERVO_FLIP
            else
                Intake.flip = Intake.GalaxyFlipPosition.SERVO_UNFLIP

        _servoflip = _gamepad.dpad_right
*/
        Intake.servoRotateVelocity =
            (_gamepad.left_trigger - _gamepad.right_trigger).toDouble() * Configs.IntakeConfig.MAX_ROTATE_VELOCITY
         */
    }
}
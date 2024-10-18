package org.firstinspires.ftc.teamcode.modules.mainControl.gamepad

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.modules.driveTrain.DriveTrain
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.modules.lift.Lift
import org.firstinspires.ftc.teamcode.modules.navigation.gyro.MergeGyro
import org.firstinspires.ftc.teamcode.utils.units.Vec2

object Gamepad : IRobotModule {
    private lateinit var _gamepad: Gamepad

    override fun init(collector: BaseCollector) {
        _gamepad = collector.robot.gamepad1
    }

    private var _promotedOld = false
    private var _clampOld = false
    private var _servoflip = false

    override fun lateUpdate() {
        DriveTrain.drivePowerDirection(
            Vec2((_gamepad.left_stick_y).toDouble(), (_gamepad.left_stick_x).toDouble()).turn(MergeGyro.rotation.angle),
            (_gamepad.right_stick_x).toDouble()
        )

        if(_gamepad.cross)
            Lift.targetPosition = Lift.LiftPosition.MIDDLE
        if(_gamepad.triangle)
            Lift.targetPosition = Lift.LiftPosition.UP
        if(_gamepad.circle)
            Lift.targetPosition = Lift.LiftPosition.DOWN

        if(_gamepad.dpad_up && !_promotedOld) {
            if (Intake.position == Intake.AdvancedPosition.SERVO_UNPROMOTED)
                Intake.position = Intake.AdvancedPosition.SERVO_PROMOTED
            else
                Intake.position = Intake.AdvancedPosition.SERVO_UNPROMOTED
        }

        _promotedOld = _gamepad.dpad_up

        if(_gamepad.dpad_down && !_clampOld)
            if(Intake.clamp == Intake.ClampPosition.SERVO_UNCLAMP)
            Intake.clamp = Intake.ClampPosition.SERVO_CLAMP
             else
                Intake.clamp = Intake.ClampPosition.SERVO_UNCLAMP

        _clampOld = _gamepad.dpad_up

        if(_gamepad.dpad_left && !_servoflip)
          if(Intake.flip == Intake.GalaxyFlipPosition.SERVO_UNFLIP)
              Intake.flip = Intake.GalaxyFlipPosition.SERVO_FLIP
           else
               Intake.flip = Intake.GalaxyFlipPosition.SERVO_UNFLIP

        _servoflip = _gamepad.dpad_left
    }
}
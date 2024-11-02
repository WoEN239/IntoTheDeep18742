package org.firstinspires.ftc.teamcode.modules.mainControl.gamepad

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.modules.driveTrain.DriveTrain
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.modules.intake.Intake.GalaxyFlipPosition
import org.firstinspires.ftc.teamcode.modules.lift.Lift
import org.firstinspires.ftc.teamcode.modules.navigation.gyro.MergeGyro
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.timer.Timer
import org.firstinspires.ftc.teamcode.utils.units.Vec2

object Gamepad : IRobotModule {
    private lateinit var _gamepad: Gamepad
    private lateinit var _lightPopit: DcMotorEx

    override fun init(collector: BaseCollector) {
        _gamepad = collector.robot.gamepad1
        Intake.rotateUp = Intake.RotatePositionUp.SERVO_ROTATEUP
        Intake.flip = GalaxyFlipPosition.SERVO_FLIP

        _lightPopit = collector.devices.lightPopit
    }

    private var _promotedOld = false
    private var _clampOld = false
    private var _servoflip = false
    private var _clampOldF = false
    private var _clampOldU = false
    private var _rotateOldU = false
    private var _liftOld = false
    private val _timer = Timer()
    private var _lightOld = false
    private var _lightOn = false

    private val _msg = arrayOf(0, 1, 0, 2, 1, 1, 1, 2, 1, 1, 2, 0, 1, 2, 0, 0, 2, 1, 0, 0, 2, 0, 1, 0, 2, 0, 2, 1, 1, 0, 2, 0, 0, 1, 2, 0, 1, 0, 0, 2, 0, 1, 0, 1, 2, 1, 2, 1, 1, 1, 2, 0, 1, 0 )
    private var _currentNumber = 0
    private val _deltaTime = ElapsedTime()

    override fun lateUpdate() {
        DriveTrain.drivePowerDirection(
            Vec2(
                (-_gamepad.left_stick_y).toDouble(),
                (-_gamepad.left_stick_x).toDouble()),
            (-_gamepad.right_stick_x).toDouble()
        )

        //  if (_gamepad.cross)
        //      Lift.targetPosition = Lift.LiftPosition.MIDDLE
        //     if (_gamepad.triangle)
        //      Lift.targetPosition = Lift.LiftPosition.UP
        //       if (_gamepad.circle)
        //       Lift.targetPosition = Lift.LiftPosition.DOWN

        if (_gamepad.dpad_up && !_promotedOld) {
            if (Intake.position == Intake.AdvancedPosition.SERVO_UNPROMOTED) {
                Intake.position = Intake.AdvancedPosition.SERVO_PROMOTED
                //  Intake.flip = Intake.GalaxyFlipPosition.SERVO_FLIP

            } else {
                Intake.position = Intake.AdvancedPosition.SERVO_UNPROMOTED
                //   Intake.flip = Intake.GalaxyFlipPosition.SERVO_UNFLIP}
            }
        }

        when(_msg[_currentNumber]) {
            0 -> if (_deltaTime.seconds() > 0.1) _lightPopit.power = 0.0
            1 -> if (_deltaTime.seconds() > 0.3) _lightPopit.power = 0.0
            2 -> {
                if(_deltaTime.seconds() > 0.5) {
                    _deltaTime.reset()
                    _lightPopit.power = 1.0

                    _currentNumber++
                    _currentNumber %= _msg.size
                }
            }
        }


        _promotedOld = _gamepad.dpad_up
        /*
        if (_gamepad.dpad_up && !_clampOld)
            if (Intake.clamp == Intake.ClampPosition.SERVO_UNCLAMP) {
                Intake.clamp = Intake.ClampPosition.SERVO_CLAMP
                Intake.clampF = Intake.ClampPositionF.SERVO_CLAMPF
                //  Intake.position = Intake.AdvancedPosition.SERVO_UNPROMOTED
                // Intake.rotateUp = Intake.RotatePositionUp.SERVO_ROTATEUP
                //   Intake.flip = Intake.GalaxyFlipPosition.SERVO_UNFLIP

                // Intake.rotateUp = Intake.RotatePositionUp.SERVO_ROTATEUP//
                /*     _timer.start(1.0) {//
                     Intake.clampUp = Intake.ClampPositionUp.SERVO_CLAMPUP//

                     _timer.start(1.0) {//
                         Intake.clamp = Intake.ClampPosition.SERVO_UNCLAMP//
                         _timer.start(1.0) {//
                             Intake.rotateUp = Intake.RotatePositionUp.SERVO_UNROTATEUP//
                         }//
                     }//
                 }//
             */
            }
            else {
                Intake.clampF = Intake.ClampPositionF.SERVO_UNCLAMPF
                //  Intake.position = Intake.AdvancedPosition.SERVO_PROMOTED
                // Intake.flip = Intake.GalaxyFlipPosition.SERVO_FLIP
                Intake.clamp = Intake.ClampPosition.SERVO_UNCLAMP
                // Intake.rotateUp = Intake.RotatePositionUp.SERVO_UNROTATEUP
            }

        _clampOld = _gamepad.dpad_up
*/
        if (_gamepad.triangle && !_clampOldU)
            if (Intake.clampUp == Intake.ClampPositionUp.SERVO_UNCLAMPUP)
                Intake.clampUp = Intake.ClampPositionUp.SERVO_CLAMPUP
            else
                Intake.clampUp = Intake.ClampPositionUp.SERVO_UNCLAMPUP

        _clampOldU = _gamepad.triangle

        // if (_gamepad.dpad_left && !_rotateOldU)
        //     if (Intake.rotateUp == Intake.RotatePositionUp.SERVO_UNROTATEUP)
        //     Intake.rotateUp = Intake.RotatePositionUp.SERVO_ROTATEUP
        // else
        //    Intake.rotateUp = Intake.RotatePositionUp.SERVO_UNROTATEUP

        //  _rotateOldU = _gamepad.dpad_left


        if (_gamepad.dpad_down && !_servoflip)
            if (Intake.flip == Intake.GalaxyFlipPosition.SERVO_UNFLIP) {//UNFLIP хаваем
                Intake.flip = Intake.GalaxyFlipPosition.SERVO_FLIP
                Intake.clamp = Intake.ClampPosition.SERVO_CLAMP
                Intake.clampF = Intake.ClampPositionF.SERVO_CLAMPF
            } else {
                Intake.clamp = Intake.ClampPosition.SERVO_UNCLAMP
                Intake.clampF = Intake.ClampPositionF.SERVO_UNCLAMPF
                Intake.flip = Intake.GalaxyFlipPosition.SERVO_UNFLIP

            }
        _servoflip = _gamepad.dpad_down
        //////////////
        if (_gamepad.cross)
             {
                Intake.flip = Intake.GalaxyFlipPosition.SERVO_UNFLIP
                _timer.start(2.0)
               {
                    Intake.clamp = Intake.ClampPosition.SERVO_UNCLAMP
                    Intake.clampF = Intake.ClampPositionF.SERVO_UNCLAMPF
                    Intake.flip = Intake.GalaxyFlipPosition.SERVO_FLIP
                }
            }
                /*  if (_gamepad.dpad_up && !_servoflip)
              if (Intake.flip == Intake.GalaxyFlipPosition.SERVO_UNFLIP) {//UNFLIP
                  Intake.flip = Intake.GalaxyFlipPosition.SERVO_FLIP
                  _timer.start(2.0) {

                      Intake.clamp = Intake.ClampPosition.SERVO_UNCLAMP
                      Intake.clampF = Intake.ClampPositionF.SERVO_UNCLAMPF
                  }
              } else {
          Intake.clamp = Intake.ClampPosition.SERVO_CLAMP
          Intake.clampF = Intake.ClampPositionF.SERVO_CLAMPF
           _timer.start(0.5) {
               Intake.flip = Intake.GalaxyFlipPosition.SERVO_UNFLIP
           }
      }
          _servoflip = _gamepad.dpad_up
  */
                Intake.servoRotateVelocity =
                    (_gamepad.left_trigger - _gamepad.right_trigger).toDouble() * Configs.IntakeConfig.MAX_ROTATE_VELOCITY

                /*if (!_liftOld && _gamepad.triangle) {
            when (Lift.targetPosition) {
                Lift.LiftPosition.DOWN -> Lift.targetPosition = Lift.LiftPosition.MIDDLE

                Lift.LiftPosition.MIDDLE -> Lift.targetPosition = Lift.LiftPosition.UP

                else -> Lift.targetPosition = Lift.LiftPosition.DOWN
            }
        }

        _liftOld = _gamepad.triangle*/

                Lift.powerUp =
                    if (_gamepad.left_bumper) Configs.LiftConfig.LIFT_POWER else if (_gamepad.right_bumper) -Configs.LiftConfig.LIFT_POWER else 0.0 }
    }

package org.firstinspires.ftc.teamcode.modules.mainControl.gamepad

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.modules.driveTrain.DriveTrain
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.modules.lift.Lift
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.devices.Devices
import org.firstinspires.ftc.teamcode.utils.units.Vec2

object Gamepad : IRobotModule {
    private lateinit var _gamepad: Gamepad

    override fun init(collector: BaseCollector) {
        _gamepad = collector.robot.gamepad1
    }

    override fun lateUpdate() {
        var promT = false
        var clampT = false
        DriveTrain.driveSimpleDirection(
            Vec2((-_gamepad.left_stick_y).toDouble(), (-_gamepad.left_stick_x).toDouble()),
            (-_gamepad.right_stick_x).toDouble()
        )

        if(_gamepad.cross)
            Lift.targetPosition = Lift.LiftPosition.MIDDLE
        if(_gamepad.triangle)
            Lift.targetPosition = Lift.LiftPosition.UP
        if(_gamepad.circle)
            Lift.targetPosition = Lift.LiftPosition.DOWN
        if(_gamepad.dpad_up)
            promT = !promT
            if(promT == true)
            Intake.position = Intake.AdvancedPosition.SERVO_PROMOTED
            else
                Intake.position = Intake.AdvancedPosition.SERVO_UNPROMOTED
        if(_gamepad.dpad_down)
            clampT = !clampT
            if(clampT == true)
            Intake.clamp = Intake.ClampPosition.SERVO_CLAMP
             else
                Intake.clamp = Intake.ClampPosition.SERVO_UNCLAMP
    }
}
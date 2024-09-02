package org.firstinspires.ftc.teamcode.modules.mainControl.gamepad

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.modules.driveTrain.DriveTrain
import org.firstinspires.ftc.teamcode.utils.units.Vec2

object Gamepad : IRobotModule {
    private lateinit var _gamepad: Gamepad

    override fun init(collector: BaseCollector) {
        _gamepad = collector.robot.gamepad1
    }

    override fun lateUpdate() {
        DriveTrain.driveDirection(
            Vec2((-_gamepad.left_stick_y).toDouble(), (-_gamepad.left_stick_x).toDouble()),
            (-_gamepad.right_stick_x).toDouble()
        )
    }
}
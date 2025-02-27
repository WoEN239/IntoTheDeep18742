package org.firstinspires.ftc.teamcode.linearOpModes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.teamcode.utils.configs.Configs

@TeleOp
class DisableTelemetry: LinearOpMode() {
    override fun runOpMode() {
        Configs.TelemetryConfig.ENABLE = false

        OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getActivity()).startActiveOpMode()

        waitForStart()

        while (opModeIsActive());
    }
}
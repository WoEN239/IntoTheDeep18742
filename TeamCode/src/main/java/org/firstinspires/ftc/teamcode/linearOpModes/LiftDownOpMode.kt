package org.firstinspires.ftc.teamcode.linearOpModes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.robotcore.internal.system.AppUtil

@TeleOp
class LiftDownOpMode: LinearOpMode() {
    override fun runOpMode() {
        val motor = hardwareMap.get("liftExtensionMotor") as DcMotorEx

        OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getActivity()).startActiveOpMode()

        waitForStart()
        resetRuntime()

        motor.power = -0.3

        while (opModeIsActive());
    }
}
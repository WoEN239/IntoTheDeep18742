package org.firstinspires.ftc.teamcode.linearOpModes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.modules.mainControl.gamepad.Gamepad

@TeleOp
class TeleOp: LinearOpModeBase() {
    override fun getCollector(): BaseCollector{
        val collector = BaseCollector(this)

        collector.addAdditionalModules(arrayOf(/*ся модули*/))

        return collector
    }
}
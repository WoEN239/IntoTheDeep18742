package org.firstinspires.ftc.teamcode.linearOpModes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.modules.hook.Hook
import org.firstinspires.ftc.teamcode.modules.mainControl.gamepad.Gamepad

@TeleOp
class TeleOpMode : LinearOpModeBase() {
    override fun getOpModeSettings() = OpModeSettings(
        isAutoStart = true,
        isPreInit = false
    )

    override fun getCollector(): BaseCollector {
        val collector = BaseCollector(
            this,
            BaseCollector.GameSettings(
                startPosition = BaseCollector.GameStartPosition.BLUE_BASKET,
                isAuto = false
            )
        )

        collector.addAdditionalModules(arrayOf(/*ся модули для телеопа*/Gamepad(), Hook()))

        return collector
    }
}
package org.firstinspires.ftc.teamcode.linearOpModes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.BaseCollector.GameStartPosition
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.ActionsRunner
import org.firstinspires.ftc.teamcode.modules.mainControl.runner.TrajectorySegmentRunner

open class AutoOpMode(val startPos: GameStartPosition): LinearOpModeBase() {
    override fun getOpModeSettings() = OpModeSettings(isAutoStart = false, isPreInit = false, preInitOpModeName = "TeleOpMode")

    override fun getCollector(): BaseCollector {
        val collector = BaseCollector(this,
            BaseCollector.GameSettings(
                startPosition = startPos,
            ),
            isAuto = true,
            mutableListOf(/*ся модули для автонома*/ TrajectorySegmentRunner(), ActionsRunner())
        )

        return collector
    }
}

@Autonomous
class AutoOpModeRedBasket: AutoOpMode(GameStartPosition.RED_BASKET)

@Autonomous
class AutoOpModeRedHuman: AutoOpMode(GameStartPosition.RED_HUMAN)

@Autonomous
class AutoOpModeBlueHuman: AutoOpMode(GameStartPosition.BLUE_HUMAN)

@Autonomous
class AutoOpModeBlueBasket: AutoOpMode(GameStartPosition.BLUE_BASKET)
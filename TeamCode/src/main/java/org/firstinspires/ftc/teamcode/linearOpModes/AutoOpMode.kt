package org.firstinspires.ftc.teamcode.linearOpModes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.BaseCollector.GameStartPosition
import org.firstinspires.ftc.teamcode.modules.mainControl.actions.ActionsRunner
import org.firstinspires.ftc.teamcode.modules.mainControl.runner.TrajectorySegmentRunner

open class AutoOpMode(val startPos: GameStartPosition): LinearOpModeBase() {
    override fun getOpModeSettings() = OpModeSettings(isAutoStart = false, isPreInit = true, preInitOpModeName = "TeleOpMode")

    override fun getCollector(): BaseCollector {
        val collector = BaseCollector(this,
            BaseCollector.GameSettings(
                startPosition = GameStartPosition.NONE,
                isAuto = true
            ))

        collector.addAdditionalModules(arrayOf(/*ся модули для автонома*/ TrajectorySegmentRunner(), ActionsRunner()))

        return collector
    }
}

@Autonomous
class AutoOpModeRedBack: AutoOpMode(GameStartPosition.RED_BACK)

@Autonomous
class AutoOpModeRedForward: AutoOpMode(GameStartPosition.RED_FORWARD)

@Autonomous
class AutoOpModeBlueForward: AutoOpMode(GameStartPosition.BLUE_FORWARD)

@Autonomous
class AutoOpModeBlueBack: AutoOpMode(GameStartPosition.BLUE_BACK)
package org.firstinspires.ftc.teamcode.linearOpModes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.modules.mainControl.runner.TrajectorySegmentRunner

@Autonomous
class AutoOpMode: LinearOpModeBase() {
    override fun getOpModeSettings() = OpModeSettings(isAutoStart = false, isPreInit = true, preInitOpModeName = "TeleOpMode")

    override fun getCollector(): BaseCollector {
        val collector = BaseCollector(this,
            BaseCollector.GameSettings(
                startPosition = BaseCollector.GameStartPosition.NONE,
                isAuto = true
            ))

        collector.addAdditionalModules(arrayOf(/*ся модули для автонома*/ TrajectorySegmentRunner()))

        return collector
    }
}
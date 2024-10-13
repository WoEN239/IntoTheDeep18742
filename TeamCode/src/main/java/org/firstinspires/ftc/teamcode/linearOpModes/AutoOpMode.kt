package org.firstinspires.ftc.teamcode.linearOpModes

import org.firstinspires.ftc.teamcode.collectors.BaseCollector

class AutoOpMode: LinearOpModeBase() {
    override fun getOpModeSettings() = OpModeSettings(isAutoStart = false, isPreInit = true, preInitOpModeName = "TeleOpMode")

    override fun getCollector(): BaseCollector {
        val collector = BaseCollector(this)

        collector.addAdditionalModules(arrayOf(/* ся модули для автонома*/))

        return collector
    }
}
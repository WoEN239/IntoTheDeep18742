package org.firstinspires.ftc.teamcode.modules.mainControl.globalEndState

import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule

object GlobalEndState: IRobotModule {
    private class State(val isEnd: ()->Boolean){

    }

    override fun init(collector: BaseCollector) {

    }


}
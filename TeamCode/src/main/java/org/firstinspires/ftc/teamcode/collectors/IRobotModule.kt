package org.firstinspires.ftc.teamcode.collectors

interface IRobotModule {
    fun init(collector: BaseCollector)
    fun lateInit(collector: BaseCollector) {}

    fun start() {}
    fun lateStart() {}

    fun update() {}
    fun lateUpdate() {}

    fun stop() {}
    fun lateStop() {}
}
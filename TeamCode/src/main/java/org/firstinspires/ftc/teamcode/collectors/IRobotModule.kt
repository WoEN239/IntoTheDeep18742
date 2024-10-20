package org.firstinspires.ftc.teamcode.collectors

/**
 * Базовый класс для всех модулей, коллекторы принимают именно эти классы.
 *
 * @author tikhonsmovzh
 */
interface IRobotModule {
    fun init(collector: BaseCollector)
    fun lateInit(collector: BaseCollector) {}

    fun start() {}
    fun lateStart() {}

    fun update() {}
    fun lateUpdate() {}

    fun stop() {}
    fun lateStop() {}

    fun reset() {}
}
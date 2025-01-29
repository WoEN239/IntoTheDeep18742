package org.firstinspires.ftc.teamcode.collectors

import org.firstinspires.ftc.teamcode.collectors.events.EventBus

/**
 * Базовый класс для всех модулей, коллекторы принимают именно эти классы.
 *
 * @author tikhonsmovzh
 */
interface IRobotModule {
    fun init(collector: BaseCollector, bus: EventBus)

    fun initUpdate() {}

    fun start() {}

    fun update() {}

    fun stop() {}
}
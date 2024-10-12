package org.firstinspires.ftc.teamcode.utils.updateListener

import org.firstinspires.ftc.teamcode.utils.devices.Battery

interface IHandler {
    fun init(bat: Battery) {}
    fun start() {}
    fun update() {}
    fun stop() {}
}

/**
 * Класс для инструментов которые требуют постоянного обновления
 *
 * @author tikhonsmovzh
 */
class UpdateHandler {
    companion object {
        private val _handlers: MutableList<IHandler> = mutableListOf()

        fun addHandler(handler: IHandler) {
            _handlers.add(handler)
        }
    }

    init {
        _handlers.clear()
    }

    fun init(bat: Battery){
        for(i in _handlers)
            i.init(bat)
    }

    fun start() {
        for (i in _handlers)
            i.start()
    }

    fun update() {
        for (i in _handlers)
            i.update()
    }

    fun stop() {
        for (i in _handlers)
            i.stop()
    }
}
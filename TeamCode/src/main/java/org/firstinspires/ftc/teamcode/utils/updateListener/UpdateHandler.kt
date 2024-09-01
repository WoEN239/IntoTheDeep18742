package org.firstinspires.ftc.teamcode.utils.updateListener

interface IHandler {
    fun start() {}
    fun update() {}
    fun stop() {}
}

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
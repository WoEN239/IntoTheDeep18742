package org.firstinspires.ftc.teamcode.utils.events

class Event<T> {
    private val _subscribers: MutableList<(T) -> Unit> = mutableListOf()

    fun sub(subscriber: (T) -> Unit) {
        _subscribers.add(subscriber)
    }

    operator fun plusAssign(subscriber: (T) -> Unit) {
        sub(subscriber)
    }

    fun invoke(parameter: T) {
        for (i in _subscribers)
            i.invoke(parameter)
    }
}
package org.firstinspires.ftc.teamcode.utils.events

class Event<T> {
    private val _subscribers: MutableList<(parameter: T) -> Void> = mutableListOf()

    fun sub(subscriber: (parameter: T) -> Void) {
        _subscribers.add(subscriber)
    }

    operator fun plusAssign(subscriber: (parameter: T) -> Void) {
        sub(subscriber)
    }

    fun invoke(parameter: T) {
        for (i in _subscribers)
            i.invoke(parameter)
    }
}
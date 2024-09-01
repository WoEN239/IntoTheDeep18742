package org.firstinspires.ftc.teamcode.utils.events

class AcceptEvent<T> {
    private val _subscribers: MutableList<(parameter: T) -> Boolean> = mutableListOf()

    fun sub(subscriber: (parameter: T) -> Boolean){
        _subscribers.add(subscriber)
    }

    operator fun plusAssign(subscriber: (parameter: T) -> Boolean){
        sub(subscriber)
    }

    fun invoke(parameter: T): Boolean{
        for(i in _subscribers)
            if(!i.invoke(parameter))
                return false

        return true
    }
}
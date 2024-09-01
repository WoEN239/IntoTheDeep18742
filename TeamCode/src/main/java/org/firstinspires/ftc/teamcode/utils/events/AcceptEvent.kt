package org.firstinspires.ftc.teamcode.utils.events

class AcceptEvent<T> {
    private val _subscribers: MutableList<(T) -> Boolean> = mutableListOf()

    fun sub(subscriber: (T) -> Boolean){
        _subscribers.add(subscriber)
    }

    operator fun plusAssign(subscriber: (T) -> Boolean){
        sub(subscriber)
    }

    fun invoke(parameter: T): Boolean{
        for(i in _subscribers)
            if(!i.invoke(parameter))
                return false

        return true
    }
}
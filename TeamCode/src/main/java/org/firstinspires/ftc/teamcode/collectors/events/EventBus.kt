package org.firstinspires.ftc.teamcode.collectors.events

import kotlin.reflect.KClass

class EventBus {
    private val _events = hashMapOf<KClass<*>, ArrayList<(IEvent) -> Unit>>()
    private val _anyCallbacks = mutableListOf<(IEvent) -> Unit>()

    fun <T: IEvent> subscribe(event: KClass<T>, callback: (T) -> Unit){
        if(_events[event] == null)
            _events[event] = arrayListOf()

        _events[event]?.add(callback as (IEvent) -> Unit)
    }

    fun <T: IEvent> invoke(event: T): T{
        for(i in _anyCallbacks)
            i.invoke(event)

        val callbacks = _events[event::class]

        if(callbacks == null)
            return event

        for(i in callbacks)
            i.invoke(event)

        return event
    }

    fun anySubscribe(callback: (IEvent) -> Unit){
        _anyCallbacks.add(callback)
    }
}
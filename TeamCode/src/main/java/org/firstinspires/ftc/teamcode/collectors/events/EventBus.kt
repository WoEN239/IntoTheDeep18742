package org.firstinspires.ftc.teamcode.collectors.events

import kotlin.reflect.KClass

class EventBus {
    private val _events = mutableMapOf<KClass<*>, ArrayList<(IEvent) -> Unit>>()

    fun <T: IEvent> subscribe(event: KClass<T>, callback: (T) -> Unit){
        if(_events[event] == null)
            _events[event] = arrayListOf()

        _events[event]?.add(callback as (IEvent) -> Unit)
    }

    fun invoke(event: IEvent){
        for(i in _events[event::class]!!)
            i.invoke(event)
    }
}
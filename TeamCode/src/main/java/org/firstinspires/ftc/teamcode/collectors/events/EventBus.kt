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
        if(_events[Any::class] != null){
            for(i in _events[Any::class]!!)
                i.invoke(event)
        }

        if(_events[event::class] == null)
            return

        for(i in _events[event::class]!!)
            i.invoke(event)
    }

    fun anySubscribe(callback: (IEvent) -> Unit){
        if(_events[Any::class] == null)
            _events[Any::class] = arrayListOf()

        _events[Any::class]?.add(callback)
    }
}
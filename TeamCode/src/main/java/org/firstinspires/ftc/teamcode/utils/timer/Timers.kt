package org.firstinspires.ftc.teamcode.utils.timer

import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry

class Timers {
    companion object{
        private val _timers = mutableListOf<Timer>()

        fun newTimer(): Timer{
            for (i in _timers){
                if(!i.isActive)
                    return i
            }

            val newTimer = Timer()

            _timers.add(newTimer)

            return newTimer
        }
    }

    fun reset(){
        _timers.clear()
    }

    fun update(){
        var activeCount = 0

        for(i in _timers) {
            i.update()

            if(i.isActive)
                activeCount++
        }

        StaticTelemetry.addData("created timers", _timers.size)
        StaticTelemetry.addData("active timers", activeCount)
    }
}
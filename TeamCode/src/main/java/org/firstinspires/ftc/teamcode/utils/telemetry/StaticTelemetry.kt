package org.firstinspires.ftc.teamcode.utils.telemetry

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import org.firstinspires.ftc.robotcore.external.Telemetry

object StaticTelemetry {
    private lateinit var _phoneTelemetry: Telemetry
    private var _telemetryPacket = TelemetryPacket()

    fun setPhoneTelemetry(telemetry: Telemetry) {
        _phoneTelemetry = telemetry
    }

    fun addLine(str: String){
        _telemetryPacket.addLine(str)
        _phoneTelemetry.addLine(str)
    }

    fun addData(name: String, obj: Any){
        _phoneTelemetry.addData(name, obj)
        _telemetryPacket.put(name, obj)
    }

    fun update(){
        _phoneTelemetry.update()
        FtcDashboard.getInstance().sendTelemetryPacket(_telemetryPacket)

        _telemetryPacket = TelemetryPacket()
    }
}
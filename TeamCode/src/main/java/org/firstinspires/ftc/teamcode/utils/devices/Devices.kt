package org.firstinspires.ftc.teamcode.utils.devices

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.VoltageSensor

class Battery (private val _voltageSensor: VoltageSensor){
    var charge = 1.0
    var voltage = 1.0

    fun update(){
        voltage = _voltageSensor.voltage

        charge = voltage / 13.0
    }
}

class Devices(hardMap: HardwareMap)  {
    val imu = hardMap.get("imu") as IMU
    val battery = Battery(hardMap.get("Control Hub") as VoltageSensor)
}
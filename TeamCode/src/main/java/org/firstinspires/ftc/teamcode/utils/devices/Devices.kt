package org.firstinspires.ftc.teamcode.utils.devices

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.utils.configs.Configs

class Battery (private val _voltageSensor: VoltageSensor){
    var charge = 1.0
    var voltage = 1.0

    fun update(){
        voltage = _voltageSensor.voltage

        charge = voltage / Configs.ChargeConfig.NOMINAL_VOLTAGE
    }
}

class Devices(hardMap: HardwareMap)  {
    val imu = hardMap.get("imu") as IMU
    val battery = Battery(hardMap.get("Control Hub") as VoltageSensor)
}
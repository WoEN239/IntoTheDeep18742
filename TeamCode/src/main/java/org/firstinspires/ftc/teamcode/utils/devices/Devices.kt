package org.firstinspires.ftc.teamcode.utils.devices

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU

class Devices {
    lateinit var imu: IMU

    fun initHardware(hardMap: HardwareMap){
        imu = hardMap.get("imu") as IMU
    }
}
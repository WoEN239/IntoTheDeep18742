package org.firstinspires.ftc.teamcode.utils.devices

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU

class Devices(hardMap: HardwareMap)  {
    val imu = hardMap.get("imu") as IMU
}
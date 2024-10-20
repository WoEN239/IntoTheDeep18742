package org.firstinspires.ftc.teamcode.utils.bulk

import com.qualcomm.hardware.lynx.LynxModule
import org.firstinspires.ftc.teamcode.utils.devices.Devices

class Bulk(val devices: Devices) {
    init {
        for (i in devices.hubs)
            i.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
    }

    fun update(){
        for (i in devices.hubs)
            i.bulkData
    }
}
package org.firstinspires.ftc.teamcode.collectors

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.modules.driveTrain
import org.firstinspires.ftc.teamcode.utils.devices.Devices

class BaseCollector (val robot: LinearOpMode) {
    val devices = Devices(robot.hardwareMap)

    private val allModules: Array<IRobotModule> = arrayOf()

    fun init(){
        for(i in allModules)
            i.init(this)

        for(i in allModules)
            i.lateInit(this)
    }

    fun start(){
        for(i in allModules)
            i.start()

        for(i in allModules)
            i.lateStart()
    }

    fun update(){
        for(i in allModules)
            i.update()

        for(i in allModules)
            i.lateUpdate()
    }

    fun stop(){
        for(i in allModules)
            i.stop()

        for(i in allModules)
            i.lateStop()
    }
}
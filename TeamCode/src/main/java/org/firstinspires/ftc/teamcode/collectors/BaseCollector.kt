package org.firstinspires.ftc.teamcode.collectors

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.utils.devices.Devices
class BaseCollector(val robot: LinearOpMode) {
    val devices = Devices(robot.hardwareMap)

    private val _allModules: Array<IRobotModule> = arrayOf()

    fun init() {
        for (i in _allModules)
            i.init(this)

        for (i in _allModules)
            i.lateInit(this)
    }

    fun start() {
        for (i in _allModules)
            i.start()

        for (i in _allModules)
            i.lateStart()
    }

    fun update() {
        for (i in _allModules)
            i.update()

        for (i in _allModules)
            i.lateUpdate()
    }

    fun stop() {
        for (i in _allModules)
            i.stop()

        for (i in _allModules)
            i.lateStop()
    }
}
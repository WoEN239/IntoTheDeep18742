package org.firstinspires.ftc.teamcode.collectors

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.modules.camera.Camera
import org.firstinspires.ftc.teamcode.modules.driveTrain.DriveTrain
import org.firstinspires.ftc.teamcode.modules.navigation.gyro.Gyro
import org.firstinspires.ftc.teamcode.utils.devices.Devices
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.updateListener.UpdateHandler

/**
 * Базовый кдасс для всех колекоторов
 *
 * @author tikhonsmovzh
 */
open class BaseCollector(val robot: LinearOpMode) {
    val devices = Devices(robot.hardwareMap)

    private val _allModules: MutableList<IRobotModule> = mutableListOf(/*ся модули*/Camera)

    private val _updateHandler = UpdateHandler()

    fun addAdditionalModules(modules: Array<IRobotModule>) = _allModules.addAll(modules)

    fun init() {
        for (i in _allModules)
            i.init(this)

        for (i in _allModules)
            i.lateInit(this)

        _updateHandler.init(devices.battery)

    }

    fun start() {
        for (i in _allModules)
            i.start()

        for (i in _allModules)
            i.lateStart()

        _updateHandler.start()
    }

    fun update() {
        StaticTelemetry.addData("runtime", System.currentTimeMillis())

        devices.battery.update()

        for (i in _allModules)
            i.update()

        for (i in _allModules)
            i.lateUpdate()

        _updateHandler.update()
    }

    fun stop() {
        for (i in _allModules)
            i.stop()

        for (i in _allModules)
            i.lateStop()

        _updateHandler.stop()
    }
}
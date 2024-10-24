package org.firstinspires.ftc.teamcode.collectors

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.modules.driveTrain.DriveTrain
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.modules.navigation.gyro.IMUGyro
import org.firstinspires.ftc.teamcode.modules.navigation.gyro.MergeGyro
import org.firstinspires.ftc.teamcode.modules.navigation.gyro.OdometerGyro
import org.firstinspires.ftc.teamcode.modules.navigation.odometry.HardwareOdometers
import org.firstinspires.ftc.teamcode.modules.navigation.odometry.MergeOdometry
import org.firstinspires.ftc.teamcode.modules.navigation.odometry.OdometersOdometry
import org.firstinspires.ftc.teamcode.utils.bulk.Bulk
import org.firstinspires.ftc.teamcode.utils.devices.Battery
import org.firstinspires.ftc.teamcode.utils.devices.Devices
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.updateListener.UpdateHandler

/**
 * Базовый кдасс для всех колекоторов
 *
 * @author tikhonsmovzh
 */
open class BaseCollector(val robot: LinearOpMode) {
    data class InitContext(val battery: Battery)

    val devices = Devices(robot.hardwareMap)

    private val _allModules: MutableList<IRobotModule> = mutableListOf(/*ся модули*/DriveTrain, HardwareOdometers, IMUGyro, OdometerGyro, MergeGyro, OdometersOdometry, MergeOdometry, Intake)

    private val _updateHandler = UpdateHandler()

    private val _bulkAdapter = Bulk(devices)

    fun addAdditionalModules(modules: Array<IRobotModule>) = _allModules.addAll(modules)

    fun init() {
        for (i in _allModules)
            i.init(this)

        for (i in _allModules)
            i.lateInit(this)

        _updateHandler.init(InitContext(devices.battery))

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

        _bulkAdapter.update()
        devices.battery.update()

        for (i in _allModules)
            i.update()

        for (i in _allModules)
            i.lateUpdate()

        _updateHandler.update()
    }

    fun stop() {
        for (i in _allModules) {
            i.stop()

            i.reset()
        }

        for (i in _allModules)
            i.lateStop()

        _updateHandler.stop()
    }
}
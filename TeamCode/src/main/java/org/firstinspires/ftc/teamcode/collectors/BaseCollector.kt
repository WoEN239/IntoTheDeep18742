package org.firstinspires.ftc.teamcode.collectors

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.modules.driveTrain.DriveTrain
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
import org.firstinspires.ftc.teamcode.utils.timer.Timers
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Vec2
import org.firstinspires.ftc.teamcode.utils.updateListener.UpdateHandler

/**
 * Базовый кдасс для всех колекоторов
 *
 * @author tikhonsmovzh
 */
open class BaseCollector(val robot: LinearOpMode, val gameSettings: GameSettings) {
    data class InitContext(val battery: Battery)

    val devices = Devices(robot.hardwareMap)

    private val _allModules: MutableList<IRobotModule> = mutableListOf(/*ся модули*/HardwareOdometers(), IMUGyro(), OdometerGyro(), MergeGyro(), OdometersOdometry(), MergeOdometry(), DriveTrain())

    private val _updateHandler = UpdateHandler()
    private val _bulkAdapter = Bulk(devices)
    private val _timers = Timers()

    private val _eventBus = EventBus()

    fun addAdditionalModules(modules: Array<IRobotModule>) = _allModules.addAll(modules)

    data class GameSettings(val startPosition: GameStartPosition = GameStartPosition.NONE, val isAuto: Boolean)

    enum class GameColor{ RED, BLUE }
    enum class GameOrientation { FORWARD, BACK }

    enum class GameStartPosition(val position: Vec2, val angle: Angle, color: GameColor, orientation: GameOrientation){
        RED_HUMAN(Vec2(-39.0, -165.0), Angle.ofDeg(90.0), GameColor.RED, GameOrientation.FORWARD),
        RED_BASKET(Vec2(39.0, -165.0), Angle.ofDeg(90.0), GameColor.RED, GameOrientation.BACK),
        BLUE_HUMAN(Vec2(-39.0, 165.0), Angle.ofDeg(-90.0), GameColor.BLUE, GameOrientation.FORWARD),
        BLUE_BASKET(Vec2(39.0, 165.0), Angle.ofDeg(-90.0), GameColor.BLUE, GameOrientation.BACK),
        NONE(Vec2.ZERO, Angle(0.0), GameColor.RED, GameOrientation.BACK)
    }

    fun init() {
        for (i in _allModules)
            i.init(this, _eventBus)

        for (i in _allModules)
            i.lateInit(this, _eventBus)

        _updateHandler.init(InitContext(devices.battery))

    }

    fun start() {
        _timers.reset()

        for (i in _allModules)
            i.start()

        for (i in _allModules)
            i.lateStart()

        _updateHandler.start()
        _deltaTime.reset()
    }

    private val _deltaTime = ElapsedTime()

    fun update() {
        StaticTelemetry.addData("runtime", System.currentTimeMillis())
        StaticTelemetry.addData("update time", _deltaTime.milliseconds())

        _deltaTime.reset()

        _timers.update()
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
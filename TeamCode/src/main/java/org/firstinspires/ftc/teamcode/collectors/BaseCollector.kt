package org.firstinspires.ftc.teamcode.collectors

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.modules.camera.Camera
import org.firstinspires.ftc.teamcode.modules.driveTrain.DriveTrain
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.modules.intake.IntakeManager
import org.firstinspires.ftc.teamcode.modules.navigation.gyro.IMUGyro
import org.firstinspires.ftc.teamcode.modules.navigation.gyro.MergeGyro
import org.firstinspires.ftc.teamcode.modules.navigation.gyro.OdometerGyro
import org.firstinspires.ftc.teamcode.modules.navigation.HardwareOdometers
import org.firstinspires.ftc.teamcode.modules.navigation.odometry.CVOdometry
import org.firstinspires.ftc.teamcode.modules.navigation.odometry.MergeOdometry
import org.firstinspires.ftc.teamcode.modules.navigation.odometry.OdometersOdometry
import org.firstinspires.ftc.teamcode.utils.bulk.Bulk
import org.firstinspires.ftc.teamcode.utils.devices.Battery
import org.firstinspires.ftc.teamcode.utils.devices.Devices
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.timer.Timers
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Orientation
import org.firstinspires.ftc.teamcode.utils.units.Vec2
import org.firstinspires.ftc.teamcode.utils.updateListener.UpdateHandler

/**
 * Базовый кдасс для всех колекоторов
 *
 * @author tikhonsmovzh
 */
open class BaseCollector(val robot: LinearOpMode, private val gameSettings: GameSettings, val isAuto: Boolean, val _allModules: MutableList<IRobotModule>) {
    companion object{
        private val staticParameters = StaticParameters(GameStartPosition.NONE)
    }

    data class StaticParameters(var oldStartPosition: GameStartPosition = GameStartPosition.NONE)

    data class InitContext(val battery: Battery)

    val devices = Devices(robot.hardwareMap)

    init {
        _allModules.addAll(arrayOf(/*ся модули*/HardwareOdometers(),
            IMUGyro(),
            OdometerGyro(),
            MergeGyro(),
            OdometersOdometry(),
            CVOdometry(),
            MergeOdometry(),
            DriveTrain(),
            IntakeManager(),
            Camera()
        ))
    }

    private val _updateHandler = UpdateHandler()
    private val _bulkAdapter = Bulk(devices)
    private val _timers = Timers()

    private val _eventBus = EventBus()

    data class GameSettings(val startPosition: GameStartPosition = GameStartPosition.NONE)

    enum class GameColor{ RED, BLUE }
    enum class GameOrientation { HUMAN, BASKET }

    enum class GameStartPosition(val position: Vec2, val angle: Angle, val color: GameColor, val orientation: GameOrientation){
        RED_HUMAN(Vec2(80.0 - 28.8, -156.5 + 1.5), Angle.ofDeg(90.0), GameColor.RED, GameOrientation.HUMAN),
        RED_BASKET(Vec2(-80.0, -156.0 + 1.5), Angle.ofDeg(90.0), GameColor.RED, GameOrientation.BASKET),
        BLUE_HUMAN(Vec2(-40.0, 156.5 - 1.5), Angle.ofDeg(-90.0), GameColor.BLUE, GameOrientation.HUMAN),
        BLUE_BASKET(Vec2(80.0, 156.5 - 1.5), Angle.ofDeg(-90.0), GameColor.BLUE, GameOrientation.BASKET),
        NONE(Vec2.ZERO, Angle(0.0), GameColor.BLUE, GameOrientation.BASKET)
    }

    fun init() {
        if(gameSettings.startPosition != GameStartPosition.NONE)
            staticParameters.oldStartPosition = gameSettings.startPosition

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

    fun initUpdate(){
        _deltaTime.reset()

        _timers.update()
        _bulkAdapter.update()
        devices.battery.update()

        for(i in _allModules)
            i.initUpdate()

        _updateHandler.update()
    }

    val parameters
        get() = staticParameters
}
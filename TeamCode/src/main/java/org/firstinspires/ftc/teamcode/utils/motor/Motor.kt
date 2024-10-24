package org.firstinspires.ftc.teamcode.utils.motor

import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.devices.Battery
import org.firstinspires.ftc.teamcode.utils.pidRegulator.PIDConfig
import org.firstinspires.ftc.teamcode.utils.pidRegulator.PIDRegulator
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.updateListener.IHandler
import org.firstinspires.ftc.teamcode.utils.updateListener.UpdateHandler

/**
 * Класс для удержания скорости мотора. Принимающий конфиг пида.
 *
 * @see MotorOnly
 * @see EncoderOnly
 * @see EncoderFix
 * @see PIDRegulator
 *
 * @author tikhonsmovzh
 */
class Motor(val motor: DcMotorEx, velocityPIDConfig: PIDConfig = Configs.MotorConfig.VELOCITY_PID, val maxVelocityTicks: Int = Configs.MotorConfig.DEFAULT_MAX_TICKS): IHandler {
    init {
        UpdateHandler.addHandler(this)
    }

    private lateinit var _battery: Battery

    override fun init(context: BaseCollector.InitContext) {
        _battery = context.battery
    }

    private val _velocityPid = PIDRegulator(velocityPIDConfig)
    val encoder = EncoderFix(motor) { it.toDouble() }

    var targetTicksVelocity = 0

    var targetPower : Double
        get() = targetTicksVelocity.toDouble() / maxVelocityTicks
        set(value) {
            targetTicksVelocity = (value * maxVelocityTicks).toInt()
        }
 
    override fun update() {
        StaticTelemetry.addData("vel", encoder.velocity)
        StaticTelemetry.addData("target", targetTicksVelocity)
        motor.power = _velocityPid.update(targetTicksVelocity - encoder.velocity, targetTicksVelocity.toDouble()) / _battery.charge
    }
}
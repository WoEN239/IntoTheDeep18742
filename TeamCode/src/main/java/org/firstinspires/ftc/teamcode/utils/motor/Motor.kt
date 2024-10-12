package org.firstinspires.ftc.teamcode.utils.motor

import com.qualcomm.robotcore.hardware.DcMotorEx
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
class Motor(val motor: DcMotorEx, velocityPIDConfig: PIDConfig = Configs.MotorConfig.VELOCITY_PID): IHandler {
    val maxVelocityTicks = 2400.0

    init {
        UpdateHandler.addHandler(this)
    }

    private lateinit var _battery: Battery

    override fun init(bat: Battery) {
        _battery = bat
    }

    private val _velocityPid = PIDRegulator(velocityPIDConfig)
    val encoder = EncoderFix(motor) { it.toDouble() }

    var targetTicksVelocity = 0.0

    var targetPower
        get() = targetTicksVelocity / maxVelocityTicks
        set(value) {
            targetTicksVelocity = value * maxVelocityTicks
        }
 
    override fun update() {
        StaticTelemetry.addData("vel", encoder.velocity)
        StaticTelemetry.addData("target", targetTicksVelocity)
        motor.power = _velocityPid.update(targetTicksVelocity - encoder.velocity, targetTicksVelocity) / _battery.charge
    }
}
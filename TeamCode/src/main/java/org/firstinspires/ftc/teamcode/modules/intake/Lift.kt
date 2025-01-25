package org.firstinspires.ftc.teamcode.modules.intake

import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.devices.Battery
import org.firstinspires.ftc.teamcode.utils.pidRegulator.PIDRegulator
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import kotlin.math.abs

class Lift {
    private lateinit var _aimMotor: DcMotorEx
    private lateinit var _extensionMotor: DcMotorEx

    private lateinit var _aimPotentiometer: AnalogInput

    private val _aimPID = PIDRegulator(Configs.LiftConfig.AIM_PID)
    private val _extensionPID = PIDRegulator(Configs.LiftConfig.EXTENSION_PID)

    private lateinit var _battery: Battery

    private var _aimErr = 0.0
    private var _extensionErr = 0.0

    var extensionVelocity = 0.0

    var aimTargetPosition = 0.0
    var extensionTargetPosition = 0.0

    fun init(collector: BaseCollector) {
        _battery = collector.devices.battery

        _aimPotentiometer = collector.devices.aimPotentiometer
        _aimMotor = collector.devices.liftAimMotor
        _extensionMotor = collector.devices.liftExtensionMotor

        _aimMotor.direction = REVERSE
        _aimMotor.zeroPowerBehavior = BRAKE
        _extensionMotor.zeroPowerBehavior = BRAKE

        if(collector.isAuto) {
            _extensionMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            _extensionMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
    }
    fun getCurrentExtensionPos() = _extensionMotor.currentPosition.toDouble()

    private val _deltaTime = ElapsedTime()

    var deltaExtension = 0.0
    private var _oldTargetAimPos = 0.0

    fun getAimPos() = _aimPotentiometer.voltage /
            Configs.LiftConfig.MAX_POTENTIOMETER_VOLTAGE * Configs.LiftConfig.MAX_POTENTIOMETER_ANGLE +
            Configs.LiftConfig.AIM_POTENTIOMETER_DIFFERENCE

    fun update() {
        deltaExtension += _deltaTime.seconds() * extensionVelocity

        deltaExtension = clamp(
            deltaExtension,
            Configs.LiftConfig.MIN_EXTENSION_POS,
            Configs.LiftConfig.MAX_EXTENSION_POS
        )

        _deltaTime.reset()

        val targetExtensionPos = extensionTargetPosition
        val targetAimPos = aimTargetPosition

        val targetDefencedAimPos: Double

        if (abs(Configs.LiftConfig.MIN_EXTENSION_POS - getCurrentExtensionPos()) < Configs.LiftConfig.EXTENSION_SENS) {
            targetDefencedAimPos = targetAimPos
            _oldTargetAimPos = targetAimPos
        }
        else
            targetDefencedAimPos = _oldTargetAimPos

        val targetDefencedExtensionPos: Double

        if(abs(targetAimPos - getAimPos()) > Configs.LiftConfig.AIM_SENS)
            targetDefencedExtensionPos = Configs.LiftConfig.MIN_EXTENSION_POS
        else
            targetDefencedExtensionPos = targetExtensionPos

        _aimErr = targetDefencedAimPos - getAimPos()
        _extensionErr = (targetDefencedExtensionPos + deltaExtension) - getCurrentExtensionPos()

        val triggerMinPower =
            if (getAimPos() > Configs.LiftConfig.TRIGET_SLOW_POS)
            Configs.LiftConfig.MAX_SPEED_DOWN
            else Configs.LiftConfig.MAX_TRIGGER_SPEED_DOWN

        val aimPower = _battery.voltageToPower(_aimPID.update(_aimErr)
            .coerceAtLeast(triggerMinPower).coerceAtMost(Configs.LiftConfig.MIN_SPEED_UP))

        val extensionPower = _battery.voltageToPower(_extensionPID.update(_extensionErr))

        _aimMotor.power = aimPower
        _extensionMotor.power = extensionPower
    }

    fun atTarget() = abs(_aimErr) < Configs.LiftConfig.AIM_SENS && abs(_extensionErr) < Configs.LiftConfig.EXTENSION_SENS

    fun start(){
        _deltaTime.reset()
    }
}
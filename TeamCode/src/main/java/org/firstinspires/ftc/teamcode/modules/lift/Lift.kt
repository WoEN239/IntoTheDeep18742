package org.firstinspires.ftc.teamcode.modules.lift

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
import com.qualcomm.robotcore.hardware.DigitalChannel
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.devices.Battery
import org.firstinspires.ftc.teamcode.utils.pidRegulator.PIDRegulator
import kotlin.math.PI
import kotlin.math.abs

class Lift: IRobotModule {
    class SetLiftTargetEvent(val targetAimAngle: Double, val targetPromoted: Double): IEvent
    class RequestLiftAtTargetEvent(var atTarget: Boolean): IEvent

    private lateinit var _aimMotor: DcMotorEx
    private lateinit var _promotedMotor: DcMotorEx

    private lateinit var _aimEndingUp: DigitalChannel

    private var _targetAngle = 0.0
    private var _targetPromoted = 0.0

    private val _aimPID = PIDRegulator(Configs.LiftConfig.AIM_PID)
    private val _promotedPID = PIDRegulator(Configs.LiftConfig.PROMOTED_PID)

    private lateinit var _battery: Battery

    private var _aimErr = 0.0
    private var _promotedErr = 0.0

    override fun init(collector: BaseCollector, bus: EventBus) {
        _battery = collector.devices.battery

        _aimMotor = collector.devices.liftAimMotor
        _promotedMotor = collector.devices.liftPromotedMotor

        _promotedMotor.direction = REVERSE

        _aimMotor.zeroPowerBehavior = BRAKE
        _promotedMotor.zeroPowerBehavior = BRAKE

        _aimMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        _aimMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        _promotedMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        _promotedMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        bus.subscribe(SetLiftTargetEvent::class){
            _targetAngle = it.targetAimAngle
            _targetPromoted = it.targetPromoted
        }

        bus.subscribe(RequestLiftAtTargetEvent::class){
            it.atTarget = abs(_aimErr) < Configs.LiftConfig.AIM_SENS && abs(_promotedErr) < Configs.LiftConfig.PROMOTED_SENS
        }
    }

    override fun update() {
        _aimErr = _targetAngle - _aimMotor.currentPosition.toDouble()
        _promotedErr = _targetPromoted - _promotedMotor.currentPosition.toDouble()

        val aimPower = _aimPID.update(_aimErr).coerceAtLeast(-abs(Configs.LiftConfig.MAX_SPEED_DOWN)) / _battery.charge
        val promotedPower = _promotedPID.update(_promotedErr) / _battery.charge

        _aimMotor.power = aimPower
        _promotedMotor.power = -promotedPower
    }

    override fun start() {
        /*_targetAngle = 0

        var a = {}

        a = {
            _targetAngle = 630

            Timers.newTimer().start(5.0){
                _targetAngle = 300

                Timers.newTimer().start(5.0){
                    _targetAngle = 0

                    Timers.newTimer().start(5.0, a)
                }
            }
        }

        Timers.newTimer().start(5.0, a)

        var b = {}

        b = {
            _targetPromoted = 2275

            Timers.newTimer().start(5.0){
                _targetPromoted = (2275 / 2.0).toInt()

                Timers.newTimer().start(5.0){
                    _targetPromoted = 0

                    Timers.newTimer().start(5.0, b)
                }
            }
        }

        Timers.newTimer().start(2.5){
            Timers.newTimer().start(5.0, b)
        }*/
    }
}
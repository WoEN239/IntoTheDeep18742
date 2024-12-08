package org.firstinspires.ftc.teamcode.modules.lift

import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
import com.qualcomm.robotcore.hardware.DigitalChannel
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.utils.devices.Battery

class Lift: IRobotModule {
    private lateinit var _aimMotor: DcMotorEx
    private lateinit var _extensionMotor: DcMotorEx

    private lateinit var _aimEndingUp: DigitalChannel
    private lateinit var _extensionEndingDown: DigitalChannel
    private lateinit var _battery: Battery

    override fun init(collector: BaseCollector, bus: EventBus) {
        _battery = collector.devices.battery

        _aimMotor = collector.devices.liftAimMotor
        _extensionMotor = collector.devices.liftExtensionMotor

        _extensionMotor.direction = REVERSE

        _aimMotor.zeroPowerBehavior = BRAKE
        _extensionMotor.zeroPowerBehavior = BRAKE

        _aimEndingUp = collector.devices.liftAimEndingUp
        _extensionEndingDown = collector.devices.liftExtensionEndingDown
    }

    override fun update() {

    }

    override fun start() {

    }
}
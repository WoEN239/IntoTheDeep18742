package org.firstinspires.ftc.teamcode.modules.lift

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DigitalChannel
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.motor.Motor
import org.firstinspires.ftc.teamcode.utils.pidRegulator.PIDRegulator

object Lift: IRobotModule {
    var targetPosition = LiftPosition.DOWN
    override fun update() {
        if(targetPosition == LiftPosition.MIDDLE) {
            motor.power = pidRegulator.update(targetPosition.position - motor.currentPosition)
        }
        else{
            if(targetPosition == LiftPosition.DOWN)
                if(endingDown.state  == false)
                     motor.power = Configs.LiftConfig.DOWN_SPEED
                else
                    motor.power = Configs.LiftConfig.DOWN_SPEEDLOW
            if(targetPosition == LiftPosition.UP)
                if(endingUp.state  == false)
                     motor.power = Configs.LiftConfig.UP_SPEED
                else
                    motor.power = Configs.LiftConfig.UP_SPEEDLOW
            }
    }
    private lateinit var motor: DcMotor
    private lateinit var endingDown: DigitalChannel
    private lateinit var endingUp: DigitalChannel
    private val pidRegulator = PIDRegulator(Configs.LiftConfig.LIFT_PID)
    override fun init(collector: BaseCollector) {
        //motor = collector.devices.liftMotor
        endingUp = collector.devices.endingUP
        endingDown = collector.devices.endingDown

    }
    enum class LiftPosition(val position: Double)
    {
        UP(60.0),
        DOWN(20.0),
        MIDDLE(40.0)
    }
}
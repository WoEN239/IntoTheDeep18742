package org.firstinspires.ftc.teamcode.utils.configs

import org.firstinspires.ftc.teamcode.utils.pidRegulator.PIDConfig

object Configs {
    internal object ChargeConfig{
        @JvmField var NOMINAL_VOLTAGE = 13.0
    }

    internal object MotorConfig{
        @JvmField var VELOCITY_PID = PIDConfig(0.1)
    }
}
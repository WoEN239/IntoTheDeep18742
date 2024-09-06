package org.firstinspires.ftc.teamcode.utils.configs

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.utils.pidRegulator.PIDConfig

object Configs {
    @Config
    internal object ChargeConfig{
        @JvmField var NOMINAL_VOLTAGE = 13.0
    }

    @Config
    internal object MotorConfig{
        @JvmField var VELOCITY_PID = PIDConfig(0.1)
    }

    @Config
    internal object DriveTrainConfig{
        @JvmField var WHEEL_DIAMETER = 1
        @JvmField var WHEEL_ENCODER_CONSTANT = 20
        @JvmField var WHEEL_CENTER_RADIUS = 1
        @JvmField var ROTATE_LAG = 1
        @JvmField var Y_LAG = 1
    }
}
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
        @JvmField var VELOCITY_PID = PIDConfig(0.0001, 0.006, 0.9, 0.00000001, 0.000375)
    }

    @Config
    internal object DriveTrainConfig{
        @JvmField var WHEEL_DIAMETER = 1.0
        @JvmField var WHEEL_ENCODER_CONSTANT = 20.0
        @JvmField var WHEEL_CENTER_RADIUS = 1.0
        @JvmField var Y_LAG = 1.0
    }

    @Config
    internal object RoadRunnerConfig{
        @JvmField var BUILDER_THREAD_COUNT = 5
        @JvmField var MAX_ROTATE_VELOCITY = 1.0
        @JvmField var MAX_ACCEL = 1.0

        @JvmField var MAX_TRANSLATION_VELOCITY = 1.0
    }
    @Config
    internal object LiftConfig{
        @JvmField var LIFT_PID = PIDConfig(1.0)
        @JvmField var DOWN_SPEED = 0.30
        @JvmField var UP_SPEED = 0.30
        @JvmField var DOWN_SPEEDLOW = 0.0
        @JvmField var UP_SPEEDLOW = 0.10
    }
}
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

        @JvmField var ROTATED_PID = PIDConfig(0.0)
        @JvmField var ROTATE_SENS = 0.1
    }
    @Config
    internal object LiftConfig{
        @JvmField var LIFT_PID = PIDConfig(1.0)
        @JvmField var DOWN_SPEED = 0.30
        @JvmField var UP_SPEED = 0.30
        @JvmField var DOWN_SPEEDLOW = 0.0
        @JvmField var UP_SPEEDLOW = 0.10
    }

    @Config
    internal object CameraConfig{
        @JvmField var BLUE_H_MIN = 101.0
        @JvmField var BLUE_S_MIN = 165.0
        @JvmField var BLUE_V_MIN = 0.0

        @JvmField var BLUE_H_MAX = 110.0
        @JvmField var BLUE_S_MAX = 255.0
        @JvmField var BLUE_V_MAX = 255.0

        @JvmField var K_SIZE_BLUE = 20.0

        @JvmField var COMPRESSION_COEF = 0.5
    }
    @Config
    internal object IntakeConfig {
        @JvmField var SERVO_PROMOTED = 20.0
        @JvmField var SERVO_UNPROMOTED = 30.0
        @JvmField var SERVO_CLAMP = 10.0
        @JvmField var SERVO_UNCLAMP = 20.0
        @JvmField var SERVO_UNFLIP = 10.0
        @JvmField var SERVO_FLIP = 20.0
    }
}
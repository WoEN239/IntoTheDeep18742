package org.firstinspires.ftc.teamcode.utils.configs

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.utils.pidRegulator.PIDConfig

object Configs {
    @Config
    internal object ChargeConfig {
        @JvmField
        var NOMINAL_VOLTAGE = 13.0
    }

    @Config
    internal object MotorConfig {
        @JvmField
        var VELOCITY_PID = PIDConfig(0.0001, 0.006, 0.9, 0.00000001, 0.000375)
    }

    @Config
    internal object DriveTrainConfig {
        @JvmField
        var WHEEL_DIAMETER = 1.0

        @JvmField
        var WHEEL_ENCODER_CONSTANT = 20.0

        @JvmField
        var WHEEL_CENTER_RADIUS = 1.0

        @JvmField
        var Y_LAG = 1.0

        @JvmField
        var VELOCITY_PIDF_FORWARD = PIDConfig(0.0)

        @JvmField
        var VELOCITY_PIDF_SIDE = PIDConfig(0.0)

        @JvmField
        var VELOCITY_PIDF_ROTATE = PIDConfig(0.0)
    }

    @Config
    internal object RoadRunnerConfig {
        @JvmField
        var BUILDER_THREAD_COUNT = 5

        @JvmField
        var MAX_ROTATE_VELOCITY = 1.0

        @JvmField
        var MAX_ACCEL = 1.0

        @JvmField
        var MAX_TRANSLATION_VELOCITY = 1.0

        @JvmField
        var ROTATED_PID = PIDConfig(0.0)

        @JvmField
        var ROTATE_SENS = 0.1
    }

    @Config
    internal object LiftConfig {
        @JvmField
        var LIFT_PID = PIDConfig(1.0)

        @JvmField
        var DOWN_SPEED = 0.30

        @JvmField
        var UP_SPEED = 0.30

        @JvmField
        var DOWN_SPEEDLOW = 0.0

        @JvmField
        var UP_SPEEDLOW = 0.10
    }

    @Config
    internal object CameraConfig {
        data class StickDetectConfig(
            @JvmField var H_MIN: Double,
            @JvmField var S_MIN: Double,
            @JvmField var V_MIN: Double,
            @JvmField var H_MAX: Double,
            @JvmField var S_MAX: Double,
            @JvmField var V_MAX: Double,
            @JvmField var ERODE_DILATE: Double,
            @JvmField var DILATE_ERODE: Double,
            @JvmField var PRECOMPRESSION: Double
        )

        @JvmField
        var BLUE_STICK_DETECT =
            StickDetectConfig(101.0, 165.0, 0.0, 110.0, 255.0, 255.0, 20.0, 35.0, 37.0)

        @JvmField
        var RED_STICK_DETECT =
            StickDetectConfig(0.0, 0.0, 0.0, 255.0, 255.0, 255.0, 20.0, 35.0, 37.0)

        @JvmField
        var DETECT_THREADS_COUNT = 5

        @JvmField
        var MIN_STICK_AREA = 5000.0
    }

    @Config
    internal object IntakeConfig {
        @JvmField
        var SERVO_PROMOTED = 20.0

        @JvmField
        var SERVO_UNPROMOTED = 30.0

        @JvmField
        var SERVO_CLAMP = 10.0

        @JvmField
        var SERVO_UNCLAMP = 20.0

        @JvmField
        var SERVO_UNFLIP = 10.0

        @JvmField
        var SERVO_FLIP = 20.0
    }

    @Config
    internal object SoftServo {
        @JvmField
        var DEFAULT_E = 8.0

        @JvmField
        var DEFAULT_W_MAX = 25.0
    }

    @Config
    internal object OdometryConfig{
        @JvmField
        var SIDE_ODOMETER_RADIUS = 1.0

        @JvmField
        var FORWARD_ODOMETER_LEFT_RADIUS = 1.0

        @JvmField
        var FORWARD_ODOMETER_RIGHT_RADIUS = 1.0

        @JvmField
        var ODOMETER_DIAMETER = 1.0

        @JvmField
        var ODOMETER_TICKS = 1
    }

    @Config
    internal object GyroscopeConfig{
        @JvmField
        var MERGE_COEF = 0.1

        @JvmField
        var USE_ODOMETRY = false

        @JvmField
        var GYRO_ITERATIONS = 5
    }
}
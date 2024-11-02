package org.firstinspires.ftc.teamcode.utils.configs

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.utils.pidRegulator.PIDConfig

object Configs {
    @Config
    internal object ChargeConfig {
        @JvmField
        var NOMINAL_VOLTAGE = 13.0

        @JvmField
        var BATTERY_UPDATE_SEC = 0.05
    }

    @Config
    internal object MotorConfig {
        @JvmField
        var VELOCITY_PID = PIDConfig(0.0001, 0.006, 0.9, 0.00000001, 0.000375)

        @JvmField
        var DEFAULT_MAX_TICKS = 2400.0
    }

    @Config
    internal object DriveTrainConfig {
        @JvmField
        var WHEEL_CENTER_RADIUS = 1.0

        @JvmField
        var Y_LAG = 1.2

        @JvmField
        var VELOCITY_PIDF_FORWARD = PIDConfig(0.0)

        @JvmField
        var VELOCITY_PIDF_SIDE = PIDConfig(0.0)

        @JvmField
        var VELOCITY_PIDF_ROTATE = PIDConfig(0.0)

        @JvmField
        var VELOSITY_SLOW_K_PROMOTED = 0.70

        @JvmField
        var VELOCITY_SLOW_K_LIFT = 0.70

        @JvmField
        var MAX_SPEED_FORWARD = 1.0

        @JvmField
        var MAX_SPEED_SIDE = 1.0

        @JvmField
        var MAX_SPEED_TURN = 1.0

        @JvmField
        var FORWARD_PID = PIDConfig(0.15)

        @JvmField
        var SIDE_PID = PIDConfig(0.15)
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
        var ROTATE_ACCEL = 0.1
    }

    @Config
    internal object LiftConfig {
        @JvmField
        var LIFT_PID = PIDConfig(0.05, d = 0.01, limitU = 1.0)
        @JvmField
        var LIFT_PID_SYNC = PIDConfig(0.001, limitU = 1.0)

        @JvmField
        var DOWN_SPEED = -0.30

        @JvmField
        var DOWN_SPEEDLOW = 0.0

        @JvmField
        var LIFT_MIDDLE_POS = 1470

        @JvmField
        var LIFT_UP_POS = 1470

        @JvmField
        var LIFT_POWER = 0.5

        @JvmField
        var LIFT_DOWN_POS = 100
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
        var SERVO_PROMOTED_LEFT = 0.72

        @JvmField
        var SERVO_UNPROMOTED_LEFT = 0.1

        @JvmField
        var SERVO_UNPROMOTED_RIGHT = 0.88

        @JvmField
        var SERVO_PROMOTED_RIGHT = 0.26

        @JvmField
        var SERVO_CLAMP = 0.45

        @JvmField
        var SERVO_UNCLAMP = 0.6

        @JvmField
        var SERVO_CLAMPUP = 0.43

        @JvmField
        var SERVO_UNCLAMPUP = 0.6

        @JvmField
        var SERVO_CLAMPF = 0.42

        @JvmField
        var SERVO_UNCLAMPF = 0.87

        @JvmField
        var SERVO_ROTATEUP = 0.7

        @JvmField
        var SERVO_UNROTATEUP = 0.7

        @JvmField
        var FLIP_VELOCITY = 0.5

        @JvmField
        var FLIP_STOP_POSITION = 0.5

        @JvmField
        var MAX_ROTATE_VELOCITY = 0.7
    }

    @Config
    internal object SoftServo {
        @JvmField
        var DEFAULT_E = 3.0

        @JvmField
        var DEFAULT_W_MAX = 20.0
    }

    @Config
    internal object ContServo {
        @JvmField
        var DEFAULT_E = 1.0

        @JvmField
        var DEFAULT_MAX_VELOCITY = 12.775810124598493
    }

    @Config
    internal object OdometryConfig{
        @JvmField
        var SIDE_ODOMETER_RADIUS = 15.0

        @JvmField
        var FORWARD_ODOMETER_LEFT_RADIUS = 15.5

        @JvmField
        var FORWARD_ODOMETER_RIGHT_RADIUS = 15.5

        @JvmField
        var ODOMETER_DIAMETER = 4.8

        @JvmField
        var ODOMETER_TICKS = 8192
    }

    @Config
    internal object GyroscopeConfig{
        @JvmField
        var MERGE_COEF = 0.5

        @JvmField
        var READ_HZ = 50.0
    }
}
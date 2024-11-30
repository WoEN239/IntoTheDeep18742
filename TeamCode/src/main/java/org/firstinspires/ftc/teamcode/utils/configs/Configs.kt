package org.firstinspires.ftc.teamcode.utils.configs

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.utils.pidRegulator.PIDConfig
import org.firstinspires.ftc.teamcode.utils.units.Vec2

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
        var VELOCITY_PIDF_FORWARD = PIDConfig(0.01, 0.003, 0.04, 0.0, 0.004, fr = 0.12)

        @JvmField
        var VELOCITY_PIDF_SIDE = PIDConfig(0.01, 0.003, 0.04, 0.0, 0.005, fr = 0.17)

        @JvmField
        var VELOCITY_PIDF_ROTATE = PIDConfig(0.09, 0.005, 0.005, 0.0, 0.135, fr = 0.13)
    }

    @Config
    internal object RoadRunnerConfig {
        @JvmField
        var MAX_ROTATE_VELOCITY = 7.9

        @JvmField
        var MAX_TRANSLATION_ACCEL = 70.0

        @JvmField   
        var MAX_TRANSLATION_VELOCITY = 130.0

        @JvmField
        var ROTATE_ACCEL = 4.8

        @JvmField
        var ROTATE_P = 0.2

        @JvmField
        var ROTATE_SENS = 0.09

        @JvmField
        var POSITION_P_X = 1.0

        @JvmField
        var POSITION_P_Y = 1.0

        @JvmField
        var POSITION_SENS_X = 1.2

        @JvmField
        var POSITION_SENS_Y = 1.2
    }

    @Config
    internal object LiftConfig {
        @JvmField
        var AIM_PID = PIDConfig(0.01, limitU = 0.5)

        @JvmField
        var EXTENSION_PID = PIDConfig(0.01)

        @JvmField
        var PROMOTED_SENS = 1.0

        @JvmField
        var AIM_SENS = 1.0

        @JvmField
        var MAX_SPEED_DOWN = 0.6

        @JvmField
        var LIFT_ENDING_POS = 700

        @JvmField
        var AIM_GAMEPAD_SENS = 300.0

        @JvmField
        var EXTENSION_GAMEPAD_SENS = 200.0

        @JvmField
        var MIN_AIM_POS = 0.0

        @JvmField
        var MAX_AIM_POS = 680.0

        @JvmField
        var MAX_EXTENSION_POS = 2275.0

        @JvmField
        var MIN_EXTENSION_POS = 0.0
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
        var SERVO_CLAMPUP = 0.45

        @JvmField
        var SERVO_UNCLAMPUP = 0.6

        @JvmField
        var SERVO_CLAMPF = 0.42

        @JvmField
        var SERVO_UNCLAMPF = 0.87

        @JvmField
        var SERVO_ROTATEUP = 0.7

        @JvmField
        var SERVO_UNROTATEUP = 0.3

        @JvmField
        var FLIP_VELOCITY = 0.25

        @JvmField
        var FLIP_STOP_POSITION = 0.5

        @JvmField
        var MAX_ROTATE_VELOCITY = 0.5
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
        var SIDE_ODOMETER_RADIUS = -13.5 //12

        @JvmField
        var FORWARD_ODOMETER_LEFT_RADIUS = 9.0

        @JvmField
        var FORWARD_ODOMETER_RIGHT_RADIUS = 9.0

        @JvmField
        var ODOMETER_DIAMETER = 4.8

        @JvmField
        var ODOMETER_TICKS = 8192
    }

    @Config
    internal object GyroscopeConfig{
        @JvmField
        var MERGE_COEF = 0.8

        @JvmField
        var READ_HZ = 50.0
    }
}
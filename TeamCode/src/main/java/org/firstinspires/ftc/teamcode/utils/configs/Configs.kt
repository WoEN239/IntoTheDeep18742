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
        var AIM_PID = PIDConfig(0.015, d = 0.00015, limitU = 1.0)

        @JvmField
        var EXTENSION_PID = PIDConfig(0.007)

        @JvmField
        var PROMOTED_SENS = 1.0

        @JvmField
        var AIM_SENS = 1.0

        @JvmField
        var MAX_SPEED_DOWN = -1.0

        @JvmField
        var LIFT_ENDING_POS = 524

        @JvmField
        var AIM_GAMEPAD_SENS = 300.0

        @JvmField
        var EXTENSION_GAMEPAD_SENS = 1500.0

        @JvmField
        var INIT_POWER = 0.2

        @JvmField
        var EXTENSION_FIX = 0.6

        internal data class LiftPosition(
            @JvmField var EXTENSION_POSITION: Double,
            @JvmField var AIM_POSITION: Double
        )

        @JvmField
        var TARGET_UP_BASKET_LIFT_POSITION = LiftPosition(0.0, 0.0)

        @JvmField
        var TARGET_MIDDLE_BASKET_LIFT_POSITION = LiftPosition(0.0, 0.0)

        @JvmField
        var TARGET_DOWN_BASKET_LIFT_POSITION = LiftPosition(0.0, 0.0)

        @JvmField
        var TARGET_UP_LAYER_LIFT_POSITION = LiftPosition(0.0, 0.0)

        @JvmField
        var TARGET_DOWN_LAYER_LIFT_POSITION = LiftPosition(0.0, 0.0)

        @JvmField
        var TARGET_CLAMP_FIELD_LIFT_POSITION = LiftPosition(0.0, 0.0)

        @JvmField
        var TARGET_CLAMP_CENTER_LIFT_POSITION = LiftPosition(0.0, 0.0)

        @JvmField
        var TARGET_CLAMP_WALL_LIFT_POSITION = LiftPosition(0.0, 0.0)

        @JvmField
        var TARGET_SETUP_LIFT_POSITION = LiftPosition(0.0, 0.0)
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
        var MAX = 180.0

        @JvmField
        var SERVO_CLAMP = 0.0

        @JvmField
        var SERVO_UNCLAMP = 0.27
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

    @Config
    internal object HookConfig{
        @JvmField
        var HOOK_SPEED = 0.5
    }
}
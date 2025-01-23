package org.firstinspires.ftc.teamcode.utils.configs

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.utils.pidRegulator.PIDConfig
import org.firstinspires.ftc.teamcode.utils.units.Color
import org.firstinspires.ftc.teamcode.utils.units.Vec2

object Configs {
    @Config
    internal object ChargeConfig {
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
        var VELOCITY_PIDF_FORWARD = PIDConfig(p = 2.0, limitI = 1.4, i = 0.7, f = 2.3, resetZeroIntegral = true)

        @JvmField
        var VELOCITY_PIDF_SIDE = PIDConfig(3.0, limitI = 1.4, i = 0.7, f = 2.3, resetZeroIntegral = true)

        @JvmField
        var VELOCITY_PIDF_ROTATE = PIDConfig(10.0, limitI = 4.0, i = 4.0, f = 50.0)

        @JvmField
        var BELT_RATIO = 20.0 * (26.0 / 19.0) //очень сомнительная вещь

        @JvmField
        var LIFT_MAX_SPEED = 0.09

        @JvmField
        var LIFT_MAX_ROTATE_SPEED = 0.5

        @JvmField
        var MAX_ROTATE_VELOCITY = 9.9

        @JvmField
        var MAX_TRANSLATION_ACCEL = 80.0

        @JvmField
        var MIN_TRANSLATION_ACCEL = -40.0

        @JvmField
        var MAX_TRANSLATION_VELOCITY = 120.0

        @JvmField
        var ROTATE_ACCEL = 4.8
    }

    @Config
    internal object RoadRunnerConfig {
        @JvmField
        var STEP_X = 2.5

        @JvmField
        var STEP_Y = 2.5

        @JvmField
        var STEP_H = 0.1

        @JvmField
        var ROTATE_P = 6.4

        @JvmField
        var ROTATE_SENS = 0.005

        @JvmField
        var POSITION_P_X = 1.0

        @JvmField
        var POSITION_P_Y = 1.0

        @JvmField
        var POSITION_SENS_X = 2.0

        @JvmField
        var POSITION_SENS_Y = 2.0

        @JvmField
        var POS_VELOCITY_SENS_X = 1.0

        @JvmField
        var POS_VELOCITY_SENS_Y = 1.0

        @JvmField
        var POS_VELOCITY_P_X = 0.07

        @JvmField
        var POS_VELOCITY_P_Y = 0.07

        @JvmField
        var HEADING_VEL_SENS = 0.3

        @JvmField
        var HEADING_VEL_P = 0.1
    }

    @Config
    internal object LiftConfig {
        @JvmField
        var UP_LAYER_UNCLAMP_AIM = 55.0

        @JvmField
        var UP_LAYER_UNCLAMP_EXTENSION = 90.0

        @JvmField
        var CLAMP_CENTER_AIM = 0.00

        @JvmField
        var UP_BASKED_AIM = 62.5

        @JvmField
        var UP_LAYER_AIM = 55.0

        @JvmField
        var TRANSPORT_AIM = 0.00

        @JvmField
        var TRANSPORT_EXTENSION = 0.00

        @JvmField
        var CLAMP_CENTER_EXTENSION = 0.00

        @JvmField
        var UP_BASKED_EXTENSION = 1700.00

        @JvmField
        var UP_LAYER_EXTENSION = 0.00

        @JvmField
        var AIM_PID = PIDConfig(0.75)

        @JvmField
        var EXTENSION_PID = PIDConfig(0.08)

        @JvmField
        var EXTENSION_SENS = 150.0

        @JvmField
        var AIM_SENS = 15.0

        @JvmField
        var TRIGET_SLOW_POS = 20.0

        @JvmField
        var MAX_TRIGGER_SPEED_DOWN = 0.0

        @JvmField
        var INIT_POWER = 0.13

        @JvmField
        var GAMEPAD_EXTENSION_SENS = 1900.0

        @JvmField
        var MAX_SPEED_DOWN = -10.0

        @JvmField
        var MIN_SPEED_UP = 6.0

        @JvmField
        var MAX_EXTENSION_POS = 1000.0

        @JvmField
        var MIN_EXTENSION_POS = 0.0

        @JvmField
        var MAX_POTENTIOMETER_ANGLE = 300.0

        @JvmField
        var MAX_POTENTIOMETER_VOLTAGE = 3.0

        @JvmField
        var AIM_POTENTIOMETER_DIFFERENCE = -22.8
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
            @JvmField var PRECOMPRESSION: Double,
            @JvmField var CONTOUR_COLOR: Color,
            @JvmField var TEXT_COLOR: Color
        )

        @JvmField
        var BLUE_STICK_DETECT =
            StickDetectConfig(101.0, 165.0, 0.0, 120.0, 255.0, 255.0, 20.0, 35.0, 37.0, Color.BLUE, Color.GREEN)

        @JvmField
        var RED_STICK_DETECT =
            StickDetectConfig(0.0, 0.0, 0.0, 255.0, 255.0, 255.0, 20.0, 35.0, 37.0, Color.RED, Color.GREEN)

        @JvmField
        var YELLOW_STICK_DETECT =
            StickDetectConfig(0.0, 0.0, 0.0, 255.0, 255.0, 255.0, 20.0, 35.0, 37.0, Color.YELLOW, Color.GREEN)

        @JvmField
        var DETECT_THREADS_COUNT = 3

        @JvmField
        var MIN_STICK_AREA = 1000.0

        @JvmField
        var COMPRESSION_COEF = 0.65
    }

    @Config
    internal object IntakeConfig {
        @JvmField
        var GEAR_RATIO = 16.0 / 28.0

        @JvmField
        var MAX = 270.0

        @JvmField
        var SERVO_CLAMP = 0.39

        @JvmField
        var SERVO_UNCLAMP = 0.75

        @JvmField
        var CLAMP_TIME = 0.5

        @JvmField
        var DOWN_TIME = 1.0

        @JvmField
        var CAMERA_CLAMP_POS_Y = 0.0

        @JvmField
        var CAMERA_CLAMP_POS_X = 0.0

        @JvmField
        var USE_CAMERA = false
    }

    @Config
    internal object SoftServo {
        @JvmField
        var DEFAULT_E = 8.0

        @JvmField
        var DEFAULT_W_MAX = 28.0
    }

    @Config
    internal object ContServo {
        @JvmField
        var DEFAULT_E = 1.0

        @JvmField
        var DEFAULT_MAX_VELOCITY = 12.775810124598493
    }

    @Config
    internal object CVOdometryConfig{
        @JvmField
        var CAMERA_POSITION = Vec2(0.0, 0.0)

        @JvmField
        var MERGE_COEF = 0.0

        @JvmField
        var USE_CAMERA = false
    }

    @Config
    internal object OdometryConfig {
        @JvmField
        var SIDE_ODOMETER_RADIUS = 6.235

        @JvmField
        var FORWARD_ODOMETER_LEFT_RADIUS = 8.415

        @JvmField
        var FORWARD_ODOMETER_RIGHT_RADIUS = 8.415

        @JvmField
        var ODOMETER_DIAMETER = 4.8

        @JvmField
        var ODOMETER_TICKS = 8192
    }

    @Config
    internal object GyroscopeConfig {
         @JvmField
        var MERGE_COEF = 0.0

        @JvmField
        var READ_HZ = 500.0
    }

    @Config
    internal object HookConfig {
        @JvmField
        var HOOK_SPEED = 0.5
    }

    @Config
    internal object TelemetryConfig{
        @JvmField
        var ENABLE = true

        @JvmField
        var SEND_HZ = 30.0

        @JvmField
        var ROBOT_SIZE = Vec2(41.5, 38.8)
    }
}
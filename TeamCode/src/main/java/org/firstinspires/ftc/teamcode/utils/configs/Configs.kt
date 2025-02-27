package org.firstinspires.ftc.teamcode.utils.configs

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.utils.pidRegulator.PIDConfig
import org.firstinspires.ftc.teamcode.utils.units.Color
import org.firstinspires.ftc.teamcode.utils.units.Vec2

object Configs {
    @Config
    internal object Lighting{
        @JvmField
        var ON_POWER = 0.1

        @JvmField
        var OFF_POWER = 0.999999
    }

    @Config
    internal object ChargeConfig {
        @JvmField
        var BATTERY_UPDATE_SEC = 1.0 / 10.0
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
        var VELOCITY_PIDF_FORWARD = PIDConfig(1.0, limitI = 4.0, i = 0.8, f = 1.0, resetZeroIntegral = true)

        @JvmField
        var VELOCITY_PIDF_SIDE = PIDConfig(1.0, limitI = 3.5, i = 0.8, f = 1.3, resetZeroIntegral = true)

        @JvmField
        var VELOCITY_PIDF_ROTATE = PIDConfig(20.0, limitI = 20.0, i = 1.0, f = 30.0)

        @JvmField
        var BELT_RATIO = (1.0 / 20.0) * (26.0 / 19.0) //очень сомнительная вещь

        @JvmField
        var LIFT_MAX_SPEED_K = 0.7

        @JvmField
        var LIFT_MAX_ROTATE_SPEED_K = 0.7

        @JvmField
        var MAX_ROTATE_VELOCITY = 5.0

        @JvmField
        var MAX_TRANSLATION_ACCEL = 95.0

        @JvmField
        var MIN_TRANSLATION_ACCEL = -95.0

        @JvmField
        var MAX_TRANSLATION_VELOCITY = 120.0

        @JvmField
        var ROTATE_ACCEL = 4.0
    }

    @Config
    internal object RoadRunnerConfig {
        @JvmField
        var ROAD_RUNNER_VELOCITY = 90.0

        @JvmField
        var STEP_X = 9.5

        @JvmField
        var STEP_Y = 9.5

        @JvmField
        var STEP_H = 0.6

        @JvmField
        var ROTATE_P = 3.6

        @JvmField
        var ROTATE_SENS = 0.005

        @JvmField
        var POSITION_P_X = 3.3

        @JvmField
        var POSITION_P_Y = 3.3

        @JvmField
        var POSITION_SENS_X = 1.0

        @JvmField
        var POSITION_SENS_Y = 1.0

        @JvmField
        var POS_VELOCITY_SENS_X = 0.01

        @JvmField
        var POS_VELOCITY_SENS_Y = 0.01

        @JvmField
        var POS_VELOCITY_P_X = 0.4

        @JvmField
        var POS_VELOCITY_P_Y = 0.4

        @JvmField
        var HEADING_VEL_SENS = 0.3

        @JvmField
        var HEADING_VEL_P = 0.0
    }

    @Config
    internal object LiftConfig {
        @JvmField
        var CLAMP_WALL_AIM_POS = 15.0

        @JvmField
        var CLAMP_WALL_EXTENSION_POS = 250.0

        @JvmField
        var CLAMP_WALL_CLAMPED_AIM_POS = 15.0

        @JvmField
        var CLAMP_WALL_CLAMPED_EXTENSION_POS = 250.0

        @JvmField
        var HUMAN_ADD_AIM_POS = 0.0

        @JvmField
        var HUMAN_ADD_EXTENSION_POS = 1000.0

        @JvmField
        var UP_LAYER_UNCLAMP_AIM = 68.0

        @JvmField
        var UP_LAYER_UNCLAMP_EXTENSION = 890.0

        @JvmField
        var CLAMP_CENTER_AIM = 0.00

        @JvmField
        var UP_BASKED_AIM = 65.0

        @JvmField
        var UP_LAYER_AIM = 68.0

        @JvmField
        var TRANSPORT_AIM = -5.00

        @JvmField
        var TRANSPORT_EXTENSION = 0.00

        @JvmField
        var CLAMP_CENTER_EXTENSION = 0.00

        @JvmField
        var UP_BASKED_EXTENSION = 1700.00

        @JvmField
        var UP_LAYER_EXTENSION = 0.00

        @JvmField
        var AIM_PID = PIDConfig(0.44, d = 0.007, limitI = 0.2, i = 0.1)

        @JvmField
        var EXTENSION_PID = PIDConfig(0.07)

        @JvmField
        var EXTENSION_SENS = 150.0

        @JvmField
        var AIM_SENS = 25.0

        @JvmField
        var TRIGET_SLOW_POS = 20.0

        @JvmField
        var MAX_TRIGGER_SPEED_DOWN = 0.0

        @JvmField
        var GAMEPAD_EXTENSION_SENS = 1900.0

        @JvmField
        var MAX_SPEED_DOWN = -9.0

        @JvmField
        var MIN_SPEED_UP = 12.0

        @JvmField
        var MAX_EXTENSION_POS = 1000.0

        @JvmField
        var MIN_EXTENSION_POS = 0.0

        @JvmField
        var MAX_POTENTIOMETER_ANGLE = 300.0

        @JvmField
        var MAX_POTENTIOMETER_VOLTAGE = 3.0

        @JvmField
        var AIM_POTENTIOMETER_DIFFERENCE = -17.8

        @JvmField
        var INIT_POS = 35.0
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
        var GAMEPADE_DIF_STEP = 20.0

        @JvmField
        var MAX_DIF_POS_Y = 80.0 - 13.0

        @JvmField
        var DIF_DIFFERENCE_X = 135.0

        @JvmField
        var DIF_DIFFERENCE_Y = 13.0

        @JvmField
        var GEAR_RATIO = 16.0 / 28.0

        @JvmField
        var MAX = 270.0

        @JvmField
        var SERVO_CLAMP = 0.385

        @JvmField
        var SERVO_UNCLAMP = 0.73

        @JvmField
        var CLAMP_TIME = 0.25

        @JvmField
        var UP_LAYER_DOWN_TIME = 0.3

        @JvmField
        var CLAMP_WALL_UP_TIME = 0.1

        @JvmField
        var CAMERA_CLAMP_POS_Y = 0.0

        @JvmField
        var CAMERA_CLAMP_POS_X = 0.0

        @JvmField
        var USE_CAMERA = false

        @JvmField
        var UP_BASKET_DOWN_TIME = 0.85

        @JvmField
        var UP_BASKET_DIF_POS_X = -40.0

        @JvmField
        var UP_BASKET_DIF_POS_Y = -180.0

        @JvmField
        var UP_BASKET_DOWN_MOVE_DIF_POS_X = 0.0

        @JvmField
        var UP_BASKET_DOWN_MOVE_DIF_POS_Y = -180.0

        @JvmField
        var TRANSPORT_DIF_POS_X = 0.0

        @JvmField
        var TRANSPORT_DIF_POS_Y = 0.0

        @JvmField
        var CLAMP_CENTER_DIF_POS_X = 90.0

        @JvmField
        var CLAMP_CENTER_DIF_POS_Y = 0.0

        @JvmField
        var UP_LAYER_DIF_POS_X = -80.0

        @JvmField
        var UP_LAYER_DIF_POS_Y = 0.0

        @JvmField
        var UP_LAYER_CLAMPED_DIF_POS_X = -80.0

        @JvmField
        var UP_LAYER_CLAMPED_DIF_POS_Y = 0.0

        @JvmField
        var HUMAN_ADD_DIF_POS_X = 90.0

        @JvmField
        var HUMAN_ADD_DIF_POS_Y = -45.0

        @JvmField
        var CLAMP_WALL_DIF_POS_X = 16.0

        @JvmField
        var CLAMP_WALL_DIF_POS_Y = 0.0

        @JvmField
        var CLAMP_WALL_CLAMPED_DIF_POS_X = -10.0

        @JvmField
        var CLAMP_WALL_CLAMPED_DIF_POS_Y = 0.0
    }

    @Config
    internal object SoftServo {
        @JvmField
        var DEFAULT_E = 12.0

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
    internal object CVOdometryConfig{
        @JvmField
        var CAMERA_POSITION = Vec2(34.0, 0.0)

        @JvmField
        var MERGE_COEF = 0.5

        @JvmField
        var USE_CAMERA = false

        @JvmField
        var DETECT_DIST = 100.0
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

        @JvmField
        var DUAL_ODOMETER = false

        @JvmField
        var ROTATE_SENS = 1e-8
    }

    @Config
    internal object GyroscopeConfig {
         @JvmField
        var MERGE_COEF = 0.5

        @JvmField
        var READ_HZ = 10.0

        @JvmField
        var USE_GYRO = true
    }

    @Config
    internal object HookConfig {
        @JvmField
        var HOOK_POWER = 1.0

        @JvmField
        var ACTIVATION_TIME_SEC = 60.0
    }

    @Config
    internal object TelemetryConfig{
        @JvmField
        var ENABLE = true

        @JvmField
        var SEND_HZ = 10.0

        @JvmField
        var ROBOT_SIZE = Vec2(41.5, 38.8)
    }
}
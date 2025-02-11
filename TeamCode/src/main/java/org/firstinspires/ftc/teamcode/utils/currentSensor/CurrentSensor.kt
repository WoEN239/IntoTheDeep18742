package org.firstinspires.ftc.teamcode.utils.currentSensor

import com.qualcomm.robotcore.hardware.AnalogInput
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import kotlin.math.sign

class CurrentSensor(val analogInput: AnalogInput,
                    val maxSensorCurrent: Double = Configs.CurrentSensor.DEFAULT_SENSOR_MAX_CURRENT,
                    val backgroundCurrent: Double = Configs.CurrentSensor.DEFAULT_BACKGROUND_CURRENT) {
    val current: Double
        get() {
            val value = (analogInput.voltage * 2.0 - 1.0) * maxSensorCurrent

            return value - backgroundCurrent * sign(value)
        }
}
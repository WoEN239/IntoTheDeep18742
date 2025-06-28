package org.firstinspires.ftc.teamcode.utils.currentSensor

import com.qualcomm.robotcore.hardware.AnalogInput
import org.firstinspires.ftc.teamcode.utils.configs.Configs

class CurrentSensor(
    val analogInput: AnalogInput,
    val ampsPerVolt: Double = Configs.CurrentSensor.DEFAULT_AMPS_PER_VOLT,
    val backgroundCurrent: Double = Configs.CurrentSensor.DEFAULT_BACKGROUND_CURRENT
) {
    val current: Double
        get() {
            val value =
                (analogInput.voltage - Configs.CurrentSensor.ANALOG_INPUT_MAX_VOLTADGE / 2.0) * ampsPerVolt

            return value - backgroundCurrent
        }
}
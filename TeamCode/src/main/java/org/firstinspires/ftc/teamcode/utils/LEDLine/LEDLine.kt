package org.firstinspires.ftc.teamcode.utils.LEDLine

import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo

class LEDLine(val port: Servo) {
    init {
        (port as PwmControl).pwmRange = PwmControl.PwmRange(0.0, 20000.0)
    }

    var power
        get() = port.position
        set(value) {
            port.position = value
        }
}
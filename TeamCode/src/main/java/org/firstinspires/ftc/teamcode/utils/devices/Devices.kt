package org.firstinspires.ftc.teamcode.utils.devices

import com.acmerobotics.roadrunner.clamp
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.utils.LEDLine.LEDLine
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.currentSensor.CurrentSensor
import org.firstinspires.ftc.teamcode.utils.motor.EncoderOnly
import org.firstinspires.ftc.teamcode.utils.motor.MotorOnly

class Battery (private val _voltageSensor: VoltageSensor){
    var currentVoltage = 1.0

    val _oldUpdateTime = ElapsedTime()

    fun voltageToPower(voltage: Double) = voltage / currentVoltage

    fun update(){
        if(_oldUpdateTime.seconds() > 1.0 / Configs.ChargeConfig.BATTERY_UPDATE_HZ) {
            currentVoltage = _voltageSensor.voltage

            _oldUpdateTime.reset()
        }
    }
}

class Devices(hardMap: HardwareMap)  {
    val imu = hardMap.get("imu") as IMU

    val battery = Battery(hardMap.get(VoltageSensor::class.java, "Control Hub"))

    //val camera = hardMap.get("Webcam 1") as WebcamName

    val hubs = hardMap.getAll(LynxModule::class.java)

    val leftForwardDrive = MotorOnly(hardMap.get("leftForwardDrive") as DcMotorEx)
    val rightForwardDrive = MotorOnly(hardMap.get("rightForwardDrive") as DcMotorEx)
    val leftBackDrive = MotorOnly(hardMap.get("leftBackDrive") as DcMotorEx)
    val rightBackDrive = MotorOnly(hardMap.get("rightBackDrive") as DcMotorEx)

    val servoClamp = hardMap.get("servoClamp") as Servo
    val servoDifLeft = hardMap.get("servoDifLeft") as ServoImplEx
    val servoDifRight = hardMap.get("servoDifRight") as ServoImplEx

    val forwardOdometerLeft = EncoderOnly(hardMap.get("rightBackDrive") as DcMotorEx)
    val forwardOdometerRight = EncoderOnly(hardMap.get("rightOdometer") as DcMotorEx)
    val sideOdometer = EncoderOnly(hardMap.get("leftForwardDrive") as DcMotorEx)

    val liftAimMotor = MotorOnly(hardMap.get("rightOdometer") as DcMotorEx)
    val liftExtensionMotor = hardMap.get("liftExtensionMotor") as DcMotorEx

    //val liftExtensionEndingDown = hardMap.get("liftExtensionEndingDown") as DigitalChannel

    val servoHookLeft = hardMap.get("servoHookLeft") as CRServo
    val servoHookRight = hardMap.get("servoHookRight") as CRServo

    val aimPotentiometer = hardMap.get("aimPotentiometer") as AnalogInput

    val leftLight = LEDLine(hardMap.get("leftLEDLine") as Servo)
    val rightLight = LEDLine(hardMap.get("rightLEDLine") as Servo)

    val clampCurrentSensor = CurrentSensor(hardMap.get("clampCurrentSensor") as AnalogInput)
}

package org.firstinspires.ftc.teamcode.utils.devices

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.motor.EncoderOnly
import org.firstinspires.ftc.teamcode.utils.motor.MotorOnly

class Battery (private val _voltageSensor: VoltageSensor){
    var charge = 1.0
    var voltage = 1.0

    val _oldUpdateTime = ElapsedTime()

    fun update(){
        if(_oldUpdateTime.seconds() > Configs.ChargeConfig.BATTERY_UPDATE_SEC) {
            voltage = _voltageSensor.voltage

            charge = voltage / Configs.ChargeConfig.NOMINAL_VOLTAGE

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

    val liftMotor = hardMap.get("liftMotor") as DcMotorEx

    val endingDown = hardMap.get("endingDown") as DigitalChannel
    val endingUP = hardMap.get("endingUp") as DigitalChannel

    val servoClamp = hardMap.get("servoClamp") as Servo
    val servoFlip = hardMap.get("servoFlip") as Servo
    val servoRotate = hardMap.get("servoRotate") as Servo

    val horizontalServoRight = hardMap.get("horizontalServoRight") as Servo
    val horizontalServoLeft = hardMap.get("horizontalServoLeft") as Servo

    val forwardOdometerLeft = EncoderOnly(hardMap.get("leftBackDrive") as DcMotorEx)
    val forwardOdometerRight = EncoderOnly(hardMap.get("rightForwardDrive") as DcMotorEx)
    val sideOdometer = EncoderOnly(hardMap.get("sideOdometer") as DcMotorEx)
}

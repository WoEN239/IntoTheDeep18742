package org.firstinspires.ftc.teamcode.modules.hook

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.modules.intake.IntakeManager
import org.firstinspires.ftc.teamcode.modules.navigation.gyro.MergeGyro
import org.firstinspires.ftc.teamcode.utils.configs.Configs

class Hook: IRobotModule {
    class HookRun: IEvent
    class HookStop: IEvent
    class HookRunRevers: IEvent

    private lateinit var _leftHook: CRServo
    private lateinit var _rightHook: CRServo

    private val _gameTimer = ElapsedTime()

    private var _useSync = false

    private lateinit var _eventBus: EventBus

    override fun init(collector: BaseCollector, bus: EventBus) {
        _eventBus = bus

        _leftHook = collector.devices.servoHookLeft
        _rightHook = collector.devices.servoHookRight

        _rightHook.direction = DcMotorSimple.Direction.REVERSE

        bus.subscribe(HookRun::class){
            if(_gameTimer.seconds() > Configs.HookConfig.ACTIVATION_TIME_SEC) {
                _useSync = true

                setHookPower(Configs.HookConfig.HOOK_POWER)
            }
        }

        bus.subscribe(HookStop::class){
            setHookPower(0.0)
        }

        bus.subscribe(HookRunRevers::class){
            if(_gameTimer.seconds() > Configs.HookConfig.ACTIVATION_TIME_SEC) {
                _useSync = true

                setHookPower(-Configs.HookConfig.HOOK_POWER)
            }
        }
    }

    fun setHookPower(power: Double) {
        if(_useSync) {
            var yAngle = _eventBus.invoke(MergeGyro.RequestMergeGyroEvent()).yRot!!

            _leftHook.power = -yAngle.angle * Configs.HookConfig.SYNC_K + power
            _rightHook.power = yAngle.angle * Configs.HookConfig.SYNC_K + power
        }
        else{
            _leftHook.power = power
            _rightHook.power = power
        }
    }

    override fun start() {
        _gameTimer.reset()
    }
}
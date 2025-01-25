package org.firstinspires.ftc.teamcode.modules.hook

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.utils.configs.Configs

class Hook: IRobotModule {
    class HookRun: IEvent
    class HookStop: IEvent
    class HookRunRevers: IEvent

    private lateinit var _leftHook: CRServo
    private lateinit var _rightHook: CRServo

    private val _gameTimer = ElapsedTime()

    override fun init(collector: BaseCollector, bus: EventBus) {
        _leftHook = collector.devices.servoHookLeft
        _rightHook = collector.devices.servoHookRight

        bus.subscribe(HookRun::class){
            if(_gameTimer.seconds() > Configs.HookConfig.ACTIVATION_TIME_SEC) {
                _rightHook.power = Configs.HookConfig.HOOK_POWER
                _leftHook.power = -Configs.HookConfig.HOOK_POWER
            }
        }

        bus.subscribe(HookStop::class){
            _rightHook.power = 0.0
            _leftHook.power = 0.0
        }

        bus.subscribe(HookRunRevers::class){
            if(_gameTimer.seconds() > Configs.HookConfig.ACTIVATION_TIME_SEC) {
                _rightHook.power = Configs.HookConfig.HOOK_POWER
                _leftHook.power = -Configs.HookConfig.HOOK_POWER
            }
        }
    }

    override fun start() {
        _gameTimer.reset()
    }
}
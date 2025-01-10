package org.firstinspires.ftc.teamcode.modules.hook

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

    private lateinit var _leftHook: Servo
    private lateinit var _rightHook: Servo

    private val _gameTimer = ElapsedTime()

    override fun init(collector: BaseCollector, bus: EventBus) {
        //_leftHook = collector.devices.servoHookLeft
        //_rightHook = collector.devices.servoHookRight

        bus.subscribe(HookRun::class){
            if(_gameTimer.seconds() > 90.0) {
                _rightHook.position = 0.5 - Configs.HookConfig.HOOK_SPEED
                _leftHook.position = 1.0 - (0.5 - Configs.HookConfig.HOOK_SPEED)
            }
        }

        bus.subscribe(HookStop::class){
            _rightHook.position = 0.5
            _leftHook.position = 0.5
        }

        bus.subscribe(HookRunRevers::class){
            if(_gameTimer.seconds() > 90.0) {
                _rightHook.position = 0.5 + Configs.HookConfig.HOOK_SPEED
                _leftHook.position = 1.0 - (0.5 + Configs.HookConfig.HOOK_SPEED)
            }
        }
    }

    override fun start() {
        _gameTimer.reset()
    }
}
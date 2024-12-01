package org.firstinspires.ftc.teamcode.modules.hook

import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.timer.Timers

class Hook: IRobotModule {
    class HookRun: IEvent
    class HookStop: IEvent

    private lateinit var _leftHook: Servo
    private lateinit var _rightHook: Servo

    override fun init(collector: BaseCollector, bus: EventBus) {
        _leftHook = collector.devices.servoHookLeft
        _rightHook = collector.devices.servoHookRight

        bus.subscribe(HookRun::class){
            _rightHook.position = 0.5 - Configs.HookConfig.HOOK_SPEED
            _leftHook.position = 0.5 - Configs.HookConfig.HOOK_SPEED
        }

        bus.subscribe(HookStop::class){
            _rightHook.position = 0.5
            _leftHook.position = 0.5
        }
    }
}
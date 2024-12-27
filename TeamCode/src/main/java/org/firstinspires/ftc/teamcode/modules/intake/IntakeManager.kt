package org.firstinspires.ftc.teamcode.modules.intake

import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.modules.driveTrain.DriveTrain
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.timer.Timers

class IntakeManager : IRobotModule {
    class EventSetClampPose(val pos: Intake.ClampPosition) : IEvent
    class EventSetLiftPose(val pos: IntakeManager.LiftPosition) : IEvent
    class EventSetExtensionVel(val vel: Double) : IEvent
    class EventSetDifVel(val vel: Double) : IEvent
    class RequestLiftPosEvent(var pos: IntakeManager.LiftPosition? = null) : IEvent
    class RequestClampPosEvent(var pos: Intake.ClampPosition? = null) : IEvent

    enum class LiftPosition {
        CLAMP_CENTER,
        UP_BASKED,
        UP_LAYER,
        TRANSPORT
    }


    private val _intake = Intake()
    private val _lift = Lift()

    private var _liftPosition = LiftPosition.TRANSPORT

    override fun init(collector: BaseCollector, bus: EventBus) {
        _lift.init(collector)
        _intake.init(collector)
        bus.subscribe(EventSetClampPose::class) {
            fun setPos() {
                _intake.clamp = it.pos
                Timers.newTimer().start(Configs.IntakeConfig.CLAMP_TIME) {
                    bus.invoke(EventSetLiftPose(LiftPosition.TRANSPORT))
                }
            }

            if (_liftPosition != LiftPosition.UP_LAYER) {
                setPos()
            } else {
                _intake.setDifPos(0.0, 0.0)

                Timers.newTimer().start(Configs.IntakeConfig.DOWN_TIME) {
                    setPos()
                }
            }
        }
        bus.subscribe(RequestClampPosEvent::class) {
            it.pos = _intake.clamp
        }
        bus.subscribe(EventSetExtensionVel::class)
        {
            if (_liftPosition == LiftPosition.CLAMP_CENTER) {
                _lift.extensionVelocity = it.vel
            } else {
                _lift.extensionVelocity = 0.0
            }
        }
        bus.subscribe(RequestLiftPosEvent::class)
        {
            it.pos = _liftPosition

        }
        bus.subscribe(EventSetDifVel::class)
        {
            if (_liftPosition == LiftPosition.CLAMP_CENTER) {
                _intake.xVelocity = it.vel
            } else {
                _intake.xVelocity = 0.0
            }
        }
        bus.subscribe(EventSetLiftPose::class) {
            _liftPosition = it.pos

            if (it.pos == LiftPosition.UP_BASKED && _intake.clamp == Intake.ClampPosition.SERVO_CLAMP) {
                _lift.aimTargetPosition = Configs.LiftConfig.UP_BASKED_AIM
                _lift.extensionTargetPosition = Configs.LiftConfig.UP_BASKED_AIM
                _intake.setDifPos(xRot = 0.0, yRot = 0.0)
            } else if (it.pos == LiftPosition.UP_LAYER && _intake.clamp == Intake.ClampPosition.SERVO_CLAMP) {
                _lift.aimTargetPosition = Configs.LiftConfig.UP_LAYER_AIM
                _lift.extensionTargetPosition = Configs.LiftConfig.UP_LAYER_EXTENSION
                _intake.setDifPos(xRot = 0.0, yRot = 0.0)
            } else if (it.pos == LiftPosition.CLAMP_CENTER && _intake.clamp == Intake.ClampPosition.SERVO_UNCLAMP) {
                _lift.aimTargetPosition = Configs.LiftConfig.CLAMP_CENTER_AIM
                _lift.extensionTargetPosition = Configs.LiftConfig.CLAMP_CENTER_EXTENSION
                _intake.setDifPos(xRot = 0.0, yRot = 0.0)
            } else {
                _lift.aimTargetPosition = Configs.LiftConfig.TRANSPORT_AIM
                _lift.extensionTargetPosition = Configs.LiftConfig.TRANSPORT_EXTENSION
                _intake.setDifPos(xRot = 0.0, yRot = 0.0)
            }
        }

    }


    override fun update() {
        _intake.update()
        _lift.update()
    }

    override fun start() {
        _intake.start()
    }

}
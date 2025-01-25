package org.firstinspires.ftc.teamcode.modules.intake

import androidx.core.math.MathUtils.clamp
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.modules.camera.Camera
import org.firstinspires.ftc.teamcode.modules.camera.Camera.RequestAllianceDetectedSticks
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.timer.Timers
import kotlin.math.PI
import kotlin.math.sqrt

class IntakeManager : IRobotModule {
    class EventSetClampPose(val pos: Intake.ClampPosition) : IEvent
    class EventSetLiftPose(val pos: LiftPosition) : IEvent
    class EventSetExtensionVel(val vel: Double) : IEvent
    class RequestLiftPosEvent(var pos: LiftPosition? = null) : IEvent
    class RequestClampPosEvent(var pos: Intake.ClampPosition? = null) : IEvent
    class NextDifPos : IEvent
    class PreviousDifPos : IEvent
    class EventSetExtensionPosition(val pos: Double) : IEvent
    class RequestLiftAtTargetEvent(var target: Boolean? = null) : IEvent
    class RequestIntakeAtTarget(var target: Boolean? = null): IEvent

    enum class LiftPosition {
        CLAMP_CENTER,
        UP_BASKED,
        UP_LAYER,
        TRANSPORT
    }

    private lateinit var _eventBus: EventBus

    private val _intake = Intake()
    private val _lift = Lift()

    private var _liftPosition = LiftPosition.TRANSPORT

    override fun initUpdate() {
        _lift.update()
    }

    override fun init(collector: BaseCollector, bus: EventBus) {
        _eventBus = bus

        _lift.init(collector)
        _intake.init(collector)

        _lift.aimTargetPosition = Configs.LiftConfig.INIT_POS

        bus.subscribe(EventSetClampPose::class) {
            fun setPos() {
                _intake.clamp = it.pos
                if (_liftPosition != LiftPosition.TRANSPORT) {
                    Timers.newTimer().start(Configs.IntakeConfig.CLAMP_TIME) {
                        bus.invoke(EventSetLiftPose(LiftPosition.TRANSPORT))
                    }
                } else
                    bus.invoke(EventSetLiftPose(LiftPosition.TRANSPORT))
            }

            if (_liftPosition != LiftPosition.UP_LAYER) {
                setPos()
            } else {
                _lift.aimTargetPosition = Configs.LiftConfig.UP_LAYER_UNCLAMP_AIM
                _lift.extensionTargetPosition = Configs.LiftConfig.UP_LAYER_UNCLAMP_EXTENSION
                _intake.setDifPos(40.0, 0.0)

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

        bus.subscribe(EventSetExtensionPosition::class) {
            if (_liftPosition == LiftPosition.CLAMP_CENTER)
                _lift.extensionTargetPosition = it.pos
        }

        bus.subscribe(RequestLiftPosEvent::class)
        {
            it.pos = _liftPosition
        }

        bus.subscribe(NextDifPos::class) {
            if (_liftPosition == LiftPosition.CLAMP_CENTER && !Configs.IntakeConfig.USE_CAMERA)
                _intake.setDifPos(_intake.xPos, clamp(_intake.yPos + 20.0, -80.0, 80.0))
        }

        bus.subscribe(PreviousDifPos::class) {
            if (_liftPosition == LiftPosition.CLAMP_CENTER && !Configs.IntakeConfig.USE_CAMERA)
                _intake.setDifPos(_intake.xPos, clamp(_intake.yPos - 20.0, -80.0, 80.0))
        }

        bus.subscribe(EventSetLiftPose::class) {
            if(_lift.atTarget()) {
                if (it.pos == LiftPosition.UP_BASKED && _intake.clamp == Intake.ClampPosition.SERVO_CLAMP && _liftPosition == LiftPosition.TRANSPORT) {
                    _lift.aimTargetPosition = Configs.LiftConfig.UP_BASKED_AIM
                    _lift.extensionTargetPosition = Configs.LiftConfig.UP_BASKED_EXTENSION
                    _intake.setDifPos(xRot = -40.0, yRot = -180.0)
                    _liftPosition = it.pos
                } else if (it.pos == LiftPosition.UP_LAYER && _intake.clamp == Intake.ClampPosition.SERVO_CLAMP && _liftPosition == LiftPosition.TRANSPORT) {
                    _lift.aimTargetPosition = Configs.LiftConfig.UP_LAYER_AIM
                    _lift.extensionTargetPosition = Configs.LiftConfig.UP_LAYER_EXTENSION
                    _intake.setDifPos(xRot = -50.0, yRot = 0.0)
                    _liftPosition = it.pos
                } else if (it.pos == LiftPosition.CLAMP_CENTER && _intake.clamp == Intake.ClampPosition.SERVO_UNCLAMP && _liftPosition == LiftPosition.TRANSPORT) {
                    _lift.aimTargetPosition = Configs.LiftConfig.CLAMP_CENTER_AIM
                    _lift.extensionTargetPosition = Configs.LiftConfig.CLAMP_CENTER_EXTENSION
                    _intake.setDifPos(xRot = 90.0, yRot = 0.0)
                    _liftPosition = it.pos
                } else if (it.pos == LiftPosition.TRANSPORT) {
                    _lift.aimTargetPosition = Configs.LiftConfig.TRANSPORT_AIM
                    _lift.extensionTargetPosition = Configs.LiftConfig.TRANSPORT_EXTENSION

                    if (_liftPosition == LiftPosition.UP_BASKED) {
                        _intake.setDifPos(xRot = 0.0, yRot = -180.0)

                        Timers.newTimer().start(Configs.IntakeConfig.UP_BASKET_DOWN_TIME) {
                            _intake.setDifPos(xRot = -80.0, yRot = 0.0)
                        }
                    } else
                        _intake.setDifPos(xRot = -80.0, yRot = 0.0)

                    _liftPosition = it.pos
                }

                _lift.deltaExtension = 0.0
            }
        }

        bus.subscribe(RequestLiftAtTargetEvent::class) {
            it.target = _lift.atTarget()
        }

        bus.subscribe(RequestIntakeAtTarget::class){
            it.target = _intake.atTarget()
        }
    }

    override fun update() {
        _intake.update()
        _lift.update()

        if (Configs.IntakeConfig.USE_CAMERA) {
            if (_liftPosition == LiftPosition.CLAMP_CENTER) {
                _eventBus.invoke(Camera.SetStickDetectEnable(true))

                val allianceSticks = _eventBus.invoke(RequestAllianceDetectedSticks()).sticks!!
                val yellowSticks = _eventBus.invoke(Camera.RequestYellowDetectedSticks()).sticks!!

                if (allianceSticks.isEmpty() && yellowSticks.isEmpty())
                    return

                var closesdStick = allianceSticks[0]
                var closesdStickL = Double.MAX_VALUE

                for (i in allianceSticks) {
                    val catetX = i.x - Configs.IntakeConfig.CAMERA_CLAMP_POS_X
                    val catetY = i.y - Configs.IntakeConfig.CAMERA_CLAMP_POS_Y
                    val l = sqrt(catetX * catetX + catetY * catetY)
                    if (l < closesdStickL) {
                        closesdStick = i
                        closesdStickL = l
                    }
                }
                _intake.setDifPos(90.0, closesdStick.angl.angle / PI * 180.0)

            } else
                _eventBus.invoke(Camera.SetStickDetectEnable(false))
        }
    }

    override fun start() {
        _intake.start()
        _lift.start()

        _intake.clamp = Intake.ClampPosition.SERVO_CLAMP
        _eventBus.invoke(EventSetLiftPose(LiftPosition.TRANSPORT))
    }
}
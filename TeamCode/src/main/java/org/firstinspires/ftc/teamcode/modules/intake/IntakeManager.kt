package org.firstinspires.ftc.teamcode.modules.intake

import androidx.core.math.MathUtils.clamp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.modules.camera.Camera
import org.firstinspires.ftc.teamcode.modules.camera.Camera.RequestAllianceDetectedSticks
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.currentSensor.CurrentSensor
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.timer.Timers
import kotlin.math.PI
import kotlin.math.abs
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
    class RequestIntakeAtTarget(var target: Boolean? = null) : IEvent
    class ClampDefendedEvent() : IEvent

    enum class LiftPosition {
        CLAMP_CENTER,
        UP_BASKED,
        UP_LAYER,
        TRANSPORT,
        HUMAN_ADD,
        CLAMP_WALL
    }

    private lateinit var _eventBus: EventBus

    private lateinit var _clampCurrentSensor: CurrentSensor

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

        _clampCurrentSensor = collector.devices.clampCurrentSensor

        var isClampBusy = false

        if (collector.isAuto)
            _lift.aimTargetPosition = Configs.LiftConfig.INIT_POS

        bus.subscribe(EventSetClampPose::class) {
            if (!isClampBusy) {
                isClampBusy = true

                when (_liftPosition) {
                    LiftPosition.HUMAN_ADD -> {
                        _intake.clamp = it.pos

                        isClampBusy = false
                    }

                    LiftPosition.UP_LAYER -> {
                        _lift.aimTargetPosition = Configs.LiftConfig.UP_LAYER_UNCLAMP_AIM
                        _lift.extensionTargetPosition =
                            Configs.LiftConfig.UP_LAYER_UNCLAMP_EXTENSION
                        _intake.setDifPos(
                            Configs.IntakeConfig.UP_LAYER_CLAMPED_DIF_POS_X,
                            Configs.IntakeConfig.UP_LAYER_CLAMPED_DIF_POS_Y
                        )

                        Timers.newTimer().start(Configs.IntakeConfig.UP_LAYER_DOWN_TIME) {
                            _intake.clamp = Intake.ClampPosition.SERVO_UNCLAMP
                            setDownState()

                            isClampBusy = false
                        }
                    }

                    LiftPosition.CLAMP_WALL -> {
                        _intake.clamp = Intake.ClampPosition.SERVO_CLAMP

                        Timers.newTimer().start({ !_intake.atTarget() }) {
                            Timers.newTimer().start(Configs.IntakeConfig.CURRENT_SENSOR_DELAY) {
                                if (_clampCurrentSensor.current > Configs.IntakeConfig.CLAMP_CURRENT ||
                                    !Configs.IntakeConfig.USE_CURRENT_SENSOR || collector.isAuto
                                ) {

                                    _lift.aimTargetPosition =
                                        Configs.LiftConfig.CLAMP_WALL_CLAMPED_AIM_POS
                                    _lift.extensionTargetPosition =
                                        Configs.LiftConfig.CLAMP_WALL_CLAMPED_EXTENSION_POS

                                    _intake.setDifPos(
                                        Configs.IntakeConfig.CLAMP_WALL_CLAMPED_DIF_POS_X,
                                        Configs.IntakeConfig.CLAMP_WALL_CLAMPED_DIF_POS_Y
                                    )

                                    Timers.newTimer()
                                        .start(Configs.IntakeConfig.CLAMP_WALL_UP_TIME) {
                                            setDownState()
                                            isClampBusy = false
                                        }
                                } else {
                                    bus.invoke(ClampDefendedEvent())

                                    _intake.clamp = Intake.ClampPosition.SERVO_UNCLAMP

                                    isClampBusy = false
                                }
                            }
                        }
                    }

                    LiftPosition.TRANSPORT -> {
                        _intake.clamp = it.pos
                        setDownState()
                        isClampBusy = false
                    }

                    LiftPosition.CLAMP_CENTER -> {
                        _intake.clamp = Intake.ClampPosition.SERVO_CLAMP

                        Timers.newTimer().start({ !_intake.atTarget() }) {
                            Timers.newTimer().start(Configs.IntakeConfig.CURRENT_SENSOR_DELAY) {
                                if ((_clampCurrentSensor.current > Configs.IntakeConfig.CLAMP_CURRENT
                                            && _clampCurrentSensor.current < Configs.IntakeConfig.CLAMP_CURRENT_TWO) ||
                                    !Configs.IntakeConfig.USE_CURRENT_SENSOR || collector.isAuto
                                    )
                                    setDownState()
                                else {
                                    _intake.clamp = Intake.ClampPosition.SERVO_UNCLAMP

                                    bus.invoke(ClampDefendedEvent())
                                }

                                isClampBusy = false
                            }
                        }
                    }

                    LiftPosition.UP_BASKED -> {
                        _intake.clamp = Intake.ClampPosition.SERVO_UNCLAMP

                        Timers.newTimer().start({ !_intake.atTarget() }){
                            setDownState()
                            isClampBusy = false
                        }
                    }
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
            if (_liftPosition == LiftPosition.CLAMP_CENTER)
                _intake.setDifPos(
                    _intake.xPos,
                    clamp(
                        _intake.yPos + Configs.IntakeConfig.GAMEPADE_DIF_STEP,
                        -Configs.IntakeConfig.MAX_DIF_POS_Y,
                        Configs.IntakeConfig.MAX_DIF_POS_Y
                    )
                )
        }

        bus.subscribe(PreviousDifPos::class) {
            if (_liftPosition == LiftPosition.CLAMP_CENTER)
                _intake.setDifPos(
                    _intake.xPos,
                    clamp(
                        _intake.yPos - Configs.IntakeConfig.GAMEPADE_DIF_STEP,
                        -Configs.IntakeConfig.MAX_DIF_POS_Y,
                        Configs.IntakeConfig.MAX_DIF_POS_Y
                    )
                )
        }

        bus.subscribe(EventSetLiftPose::class) {
            if ((_lift.atTarget() || collector.isAuto) && !isClampBusy) {
                if (it.pos == LiftPosition.UP_BASKED && _intake.clamp == Intake.ClampPosition.SERVO_CLAMP && _liftPosition == LiftPosition.TRANSPORT) {
                    _lift.aimTargetPosition = Configs.LiftConfig.UP_BASKED_AIM
                    _lift.extensionTargetPosition = Configs.LiftConfig.UP_BASKED_EXTENSION
                    _intake.setDifPos(
                        xRot = Configs.IntakeConfig.UP_BASKET_DIF_POS_X,
                        yRot = Configs.IntakeConfig.UP_BASKET_DIF_POS_Y
                    )
                    _liftPosition = it.pos
                    _lift.deltaExtension = 0.0
                } else if (it.pos == LiftPosition.UP_LAYER && _intake.clamp == Intake.ClampPosition.SERVO_CLAMP && _liftPosition == LiftPosition.TRANSPORT) {
                    _lift.aimTargetPosition = Configs.LiftConfig.UP_LAYER_AIM
                    _lift.extensionTargetPosition = Configs.LiftConfig.UP_LAYER_EXTENSION
                    _intake.setDifPos(
                        xRot = Configs.IntakeConfig.UP_LAYER_DIF_POS_X,
                        yRot = Configs.IntakeConfig.UP_LAYER_DIF_POS_Y
                    )
                    _liftPosition = it.pos
                    _lift.deltaExtension = 0.0
                } else if (it.pos == LiftPosition.CLAMP_CENTER && _intake.clamp == Intake.ClampPosition.SERVO_UNCLAMP && _liftPosition == LiftPosition.TRANSPORT) {
                    _lift.aimTargetPosition = Configs.LiftConfig.CLAMP_CENTER_AIM
                    _lift.extensionTargetPosition = Configs.LiftConfig.CLAMP_CENTER_EXTENSION
                    _intake.setDifPos(
                        xRot = Configs.IntakeConfig.CLAMP_CENTER_DIF_POS_X,
                        yRot = Configs.IntakeConfig.CLAMP_CENTER_DIF_POS_Y
                    )
                    _liftPosition = it.pos
                    _lift.deltaExtension = 0.0
                    _cameraEnableTimer.reset()
                } else if (it.pos == LiftPosition.CLAMP_WALL && _liftPosition == LiftPosition.TRANSPORT && _intake.clamp == Intake.ClampPosition.SERVO_UNCLAMP) {
                    _lift.aimTargetPosition = Configs.LiftConfig.CLAMP_WALL_AIM_POS
                    _lift.extensionTargetPosition = Configs.LiftConfig.CLAMP_WALL_EXTENSION_POS
                    _intake.setDifPos(
                        xRot = Configs.IntakeConfig.CLAMP_WALL_DIF_POS_X,
                        yRot = Configs.IntakeConfig.CLAMP_WALL_DIF_POS_Y
                    )
                    _liftPosition = it.pos
                    _lift.deltaExtension = 0.0
                } else if (it.pos == LiftPosition.HUMAN_ADD && _liftPosition == LiftPosition.TRANSPORT) {
                    _lift.aimTargetPosition = Configs.LiftConfig.HUMAN_ADD_AIM_POS
                    _lift.extensionTargetPosition = Configs.LiftConfig.HUMAN_ADD_EXTENSION_POS
                    _intake.setDifPos(
                        xRot = Configs.IntakeConfig.HUMAN_ADD_DIF_POS_X,
                        yRot = Configs.IntakeConfig.HUMAN_ADD_DIF_POS_Y
                    )
                    _liftPosition = it.pos
                    _lift.deltaExtension = 0.0
                } else if (it.pos == LiftPosition.TRANSPORT)
                    setDownState()
            }
        }

        bus.subscribe(RequestLiftAtTargetEvent::class) {
            it.target = _lift.atTarget() && !isClampBusy
        }

        bus.subscribe(RequestIntakeAtTarget::class) {
            it.target = _intake.atTarget() && !isClampBusy
        }
    }

    private val _cameraUpdateTimer = ElapsedTime()
    private val _cameraEnableTimer = ElapsedTime()

    override fun update() {
        _lift.update()

        StaticTelemetry.addData("clamp current", _clampCurrentSensor.current)

        if (Configs.IntakeConfig.USE_CAMERA) {
            if (_liftPosition == LiftPosition.CLAMP_CENTER) {
                if (_cameraUpdateTimer.seconds() < 1.0 / Configs.IntakeConfig.CAMERA_UPDATE_HZ || _cameraEnableTimer.seconds() < Configs.IntakeConfig.CAMERA_ENABLE_TIMER)
                    return

                _cameraUpdateTimer.reset()

                _eventBus.invoke(Camera.SetStickDetectEnable(true))

                val allianceSticks = _eventBus.invoke(RequestAllianceDetectedSticks()).sticks!!
                val yellowSticks = _eventBus.invoke(Camera.RequestYellowDetectedSticks()).sticks!!

                if (allianceSticks.isEmpty() && yellowSticks.isEmpty()) {
                    //_intake.setDifPos(Configs.IntakeConfig.CLAMP_CENTER_DIF_POS_X, 0.0)
                    return
                }

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

                val rot = closesdStick.angl.toDegree()

                if (abs(rot) > Configs.IntakeConfig.CAMERA_SENS)
                    _intake.setDifPos(
                        Configs.IntakeConfig.CLAMP_CENTER_DIF_POS_X, clamp(
                            _intake.yPos + rot,
                            -Configs.IntakeConfig.MAX_DIF_POS_Y,
                            Configs.IntakeConfig.MAX_DIF_POS_Y
                        )
                    )
            } else
                _eventBus.invoke(Camera.SetStickDetectEnable(false))
        }
    }

    fun setDownState() {
        _lift.aimTargetPosition = Configs.LiftConfig.TRANSPORT_AIM
        _lift.extensionTargetPosition = Configs.LiftConfig.TRANSPORT_EXTENSION

        if (_liftPosition == LiftPosition.UP_BASKED) {
            _intake.setDifPos(
                xRot = Configs.IntakeConfig.UP_BASKET_DOWN_MOVE_DIF_POS_X,
                yRot = Configs.IntakeConfig.UP_BASKET_DOWN_MOVE_DIF_POS_Y
            )

            Timers.newTimer().start(Configs.IntakeConfig.UP_BASKET_DOWN_TIME) {
                _intake.setDifPos(
                    xRot = Configs.IntakeConfig.TRANSPORT_DIF_POS_X,
                    yRot = Configs.IntakeConfig.TRANSPORT_DIF_POS_Y
                )
            }
        } else
            _intake.setDifPos(
                xRot = Configs.IntakeConfig.TRANSPORT_DIF_POS_X,
                yRot = Configs.IntakeConfig.TRANSPORT_DIF_POS_Y
            )

        _liftPosition = LiftPosition.TRANSPORT

        _lift.deltaExtension = 0.0
    }

    override fun start() {
        _lift.start()

        _intake.clamp = Intake.ClampPosition.SERVO_CLAMP

        setDownState()
    }
}
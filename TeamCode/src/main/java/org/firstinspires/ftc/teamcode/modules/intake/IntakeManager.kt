package org.firstinspires.ftc.teamcode.modules.intake

import androidx.core.math.MathUtils.clamp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.modules.camera.Camera
import org.firstinspires.ftc.teamcode.modules.camera.Camera.RequestAllianceDetectedSticks
import org.firstinspires.ftc.teamcode.modules.driveTrain.DriveTrain
import org.firstinspires.ftc.teamcode.modules.navigation.gyro.MergeGyro
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.currentSensor.CurrentSensor
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.timer.Timers
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Orientation
import org.firstinspires.ftc.teamcode.utils.units.Vec2
import java.lang.Math.atan2
import java.lang.Math.pow
import java.lang.Math.toDegrees
import java.lang.Math.toRadians
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.tan

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
    class AutoClamp() : IEvent

    enum class LiftPosition {
        CLAMP_CENTER,
        UP_BASKED,
        UP_LAYER,
        TRANSPORT,
        HUMAN_ADD,
        CLAMP_WALL,
        LOW_BASKET,
        AUTO_CLAMP_CENTER
    }

    private lateinit var _eventBus: EventBus

    private lateinit var _clampCurrentSensor: CurrentSensor

    private val _intake = Intake()
    private val _lift = Lift()

    private var _isAuto = false

    private var _liftPosition = LiftPosition.TRANSPORT

    override fun initUpdate() {
        if (_isAuto)
            _lift.update()
    }

    override fun init(collector: BaseCollector, bus: EventBus) {
        _isAuto = collector.isAuto

        _eventBus = bus

        _lift.init(collector)
        _intake.init(collector)

        _clampCurrentSensor = collector.devices.clampCurrentSensor

        var isClampBusy = false
        var integrations = 0

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
                                    !Configs.IntakeConfig.USE_CURRENT_SENSOR || collector.isAuto ||
                                    integrations >= Configs.IntakeConfig.MAX_DEFENDED_INTEGRATIOS
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
                                    integrations++
                                }
                            }
                        }
                    }

                    LiftPosition.TRANSPORT -> {
                        _intake.clamp = it.pos
                        setDownState()
                        isClampBusy = false
                    }

                    LiftPosition.AUTO_CLAMP_CENTER -> {
                        _liftPosition = LiftPosition.TRANSPORT
                        _lift.deltaExtension = 0.0

                        Timers.newTimer().start({ !_lift.atTarget() }) {
                            Timers.newTimer()
                                .start(Configs.AutoClamp.AUTO_CLAMP_CENTER_CLAMP_TIMER) {
                                    _intake.clamp = Intake.ClampPosition.SERVO_CLAMP
                                    Timers.newTimer().start({ !_intake.atTarget() }) {
                                        isClampBusy = false
                                        setDownState()
                                    }
                                }
                        }
                    }

                    LiftPosition.CLAMP_CENTER -> {
                        _intake.clamp = Intake.ClampPosition.SERVO_CLAMP

                        Timers.newTimer().start({ !_intake.atTarget() }) {
                            Timers.newTimer().start(Configs.IntakeConfig.CURRENT_SENSOR_DELAY) {
                                if ((_clampCurrentSensor.current > Configs.IntakeConfig.CLAMP_CURRENT
                                            && _clampCurrentSensor.current < Configs.IntakeConfig.CLAMP_CURRENT_TWO) ||
                                    !Configs.IntakeConfig.USE_CURRENT_SENSOR || collector.isAuto ||
                                    integrations >= Configs.IntakeConfig.MAX_DEFENDED_INTEGRATIOS
                                )
                                    setDownState()
                                else {
                                    _intake.clamp = Intake.ClampPosition.SERVO_UNCLAMP

                                    integrations++

                                    bus.invoke(ClampDefendedEvent())
                                }

                                isClampBusy = false
                            }
                        }
                    }

                    LiftPosition.UP_BASKED, LiftPosition.LOW_BASKET -> {
                        _intake.clamp = Intake.ClampPosition.SERVO_UNCLAMP

                        Timers.newTimer().start({ !_intake.atTarget() }) {
                            Timers.newTimer().start(Configs.LiftConfig.BASKET_DELAY) {
                                setDownState()
                                isClampBusy = false
                            }
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
            if (_liftPosition == LiftPosition.CLAMP_CENTER || _liftPosition == LiftPosition.AUTO_CLAMP_CENTER) {
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
            if (_liftPosition == LiftPosition.CLAMP_CENTER || _liftPosition == LiftPosition.AUTO_CLAMP_CENTER)
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
            if (_liftPosition == LiftPosition.CLAMP_CENTER || _liftPosition == LiftPosition.AUTO_CLAMP_CENTER)
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
                if (it.pos == LiftPosition.UP_BASKED && _intake.clamp == Intake.ClampPosition.SERVO_CLAMP && (_liftPosition == LiftPosition.TRANSPORT || _liftPosition == LiftPosition.LOW_BASKET)) {
                    _lift.aimTargetPosition = Configs.LiftConfig.UP_BASKED_AIM
                    _lift.extensionTargetPosition = Configs.LiftConfig.UP_BASKED_EXTENSION
                    _intake.setDifPos(
                        xRot = Configs.IntakeConfig.UP_BASKET_DIF_POS_X,
                        yRot = Configs.IntakeConfig.UP_BASKET_DIF_POS_Y
                    )
                    _liftPosition = it.pos
                    _lift.deltaExtension = 0.0
                } else if (it.pos == LiftPosition.LOW_BASKET && _intake.clamp == Intake.ClampPosition.SERVO_CLAMP && (_liftPosition == LiftPosition.TRANSPORT || _liftPosition == LiftPosition.UP_BASKED)) {
                    _lift.aimTargetPosition = Configs.LiftConfig.LOW_BASKED_AIM
                    _lift.extensionTargetPosition = Configs.LiftConfig.LOW_BASKED_EXTENSION
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
                    integrations = 0
                } else if (it.pos == LiftPosition.CLAMP_WALL && _liftPosition == LiftPosition.TRANSPORT && _intake.clamp == Intake.ClampPosition.SERVO_UNCLAMP) {
                    _lift.aimTargetPosition = Configs.LiftConfig.CLAMP_WALL_AIM_POS
                    _lift.extensionTargetPosition = Configs.LiftConfig.CLAMP_WALL_EXTENSION_POS
                    _intake.setDifPos(
                        xRot = Configs.IntakeConfig.CLAMP_WALL_DIF_POS_X,
                        yRot = Configs.IntakeConfig.CLAMP_WALL_DIF_POS_Y
                    )
                    _liftPosition = it.pos
                    _lift.deltaExtension = 0.0
                    integrations = 0
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

        bus.subscribe(AutoClamp::class) {
            _liftPosition = LiftPosition.AUTO_CLAMP_CENTER
            _lift.aimTargetPosition = 45.0
            _lift.extensionTargetPosition = 0.0
            _intake.setDifPos(
                xRot = Configs.IntakeConfig.CLAMP_CENTER_DIF_POS_X,
                yRot = Configs.IntakeConfig.CLAMP_CENTER_DIF_POS_Y
            )

            _liftPosition = LiftPosition.AUTO_CLAMP_CENTER
            _intake.clamp = Intake.ClampPosition.SERVO_UNCLAMP

            _isCameraDetected = false

            Timers.newTimer().start({ !_lift.atTarget() }) {
                Timers.newTimer().start(Configs.AutoClamp.CAMERA_ENABLE_TIMER) {
                    bus.invoke(Camera.WaitFrameProcessed())

                    val yellowSticks = bus.invoke(Camera.RequestYellowDetectedSticks()).sticks!!
                    val allianceSticks = bus.invoke(RequestAllianceDetectedSticks()).sticks!!

                    val sticks = yellowSticks + allianceSticks

                    var minX = Double.MAX_VALUE
                    var minY = Double.MAX_VALUE

                    var closesStickL = Double.MAX_VALUE

                    for (i in sticks) {
                        val yAngle =
                            90.0 - (180.0 - 90.0 - _lift.currentAimPos) + (Configs.AutoClamp.FRAME_SIZE.y / 2.0 - i.y) * Configs.AutoClamp.PIXEL_TO_ANGLE
                        val xAngle =
                            (Configs.AutoClamp.FRAME_SIZE.x / 2.0 - i.x) * Configs.AutoClamp.PIXEL_TO_ANGLE

                        val cameraH =
                            sin(toRadians(_lift.currentAimPos)) * Configs.AutoClamp.EXTENSION_LENGHT + Configs.AutoClamp.LIFT_H
                        val cameraX =
                            cos(toRadians(_lift.currentAimPos)) * Configs.AutoClamp.EXTENSION_LENGHT

                        val xPos = cameraH * tan(toRadians(yAngle)) + cameraX
                        val yPos = cameraH * tan(toRadians(xAngle))

                        val l = sqrt(xPos * xPos + yPos * yPos)

                        if (closesStickL > l) {
                            closesStickL = l

                            _closesStickPos = Vec2(xPos, yPos)
                        }
                    }

                    _clampStartRot = _eventBus.invoke(MergeGyro.RequestMergeGyroEvent()).rotation!!

                    _isCameraDetected = true
                }
            }
        }
    }

    private var _closesStickPos = Vec2.ZERO
    private var _isCameraDetected = false
    private var _clampStartRot = Angle.ZERO

    override fun update() {
        StaticTelemetry.addData("closes stick", _closesStickPos)

        _lift.update()

        StaticTelemetry.addData(
            "liftTargetExtensionPos",
            _lift.extensionTargetPosition + _lift.deltaExtension
        )
        StaticTelemetry.addData("clamp current", _clampCurrentSensor.current)

        if (Configs.AutoClamp.ENABLE_AUTO_CLAMP && _liftPosition == LiftPosition.AUTO_CLAMP_CENTER && _isCameraDetected) {
            val targetAngle = Angle(kotlin.math.atan2(_closesStickPos.y, _closesStickPos.x))
            val err = (targetAngle - (_eventBus.invoke(MergeGyro.RequestMergeGyroEvent()).rotation!! - _clampStartRot)).angle

            _eventBus.invoke(
                DriveTrain.SetDriveCmEvent(
                    Vec2.ZERO,
                    err * Configs.AutoClamp.DRIVE_ROTATE_P
                )
            )

            _lift.aimTargetPosition = Configs.LiftConfig.CLAMP_CENTER_AIM
            _lift.extensionTargetPosition = (_closesStickPos.length() - Configs.AutoClamp.EXTENSION_LENGHT) / Configs.AutoClamp.LIFT_CIRCLE_R / PI * (Configs.AutoClamp.MOTOR_TICKS / 2.0)
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
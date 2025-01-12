package org.firstinspires.ftc.teamcode.modules.navigation.odometry

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.modules.camera.Camera.AddCameraProcessor
import org.firstinspires.ftc.teamcode.modules.intake.IntakeManager
import org.firstinspires.ftc.teamcode.modules.navigation.gyro.MergeGyro
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.units.Vec2
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

class CVOdometry : IRobotModule {
    class UpdateCVOdometryEvent(val pos: Vec2) : IEvent

    private lateinit var _aprilTagProcessor: AprilTagProcessor

    private lateinit var _eventBus: EventBus

    override fun init(collector: BaseCollector, bus: EventBus) {
        _eventBus = bus

        if (Configs.IntakeConfig.USE_CAMERA) {
            _aprilTagProcessor =
                AprilTagProcessor.Builder().setOutputUnits(DistanceUnit.CM, AngleUnit.RADIANS)
                    .build()

            bus.invoke(AddCameraProcessor(_aprilTagProcessor))
        }
    }

    override fun update() {
        if (_eventBus.invoke(IntakeManager.RequestLiftPosEvent()).pos != IntakeManager.LiftPosition.TRANSPORT ||
            !Configs.IntakeConfig.USE_CAMERA ||
            !_eventBus.invoke(IntakeManager.RequestLiftAtTargetEvent()).target!!)
            return

        val detections = _aprilTagProcessor.detections

        if (detections.isEmpty())
            return

        var sum = Vec2.ZERO

        for (i in detections) {
            val pos = i.robotPose.position

            sum += Vec2(
                pos.x,
                pos.y
            ) - Configs.CVOdometryConfig.CAMERA_POSITION.turn(-_eventBus.invoke(MergeGyro.RequestMergeGyroEvent()).rotation!!.angle)
        }

        _eventBus.invoke(UpdateCVOdometryEvent(sum / Vec2(detections.size.toDouble())))
    }
}
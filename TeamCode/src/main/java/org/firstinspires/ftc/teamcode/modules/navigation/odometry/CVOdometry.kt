package org.firstinspires.ftc.teamcode.modules.navigation.odometry

import org.firstinspires.ftc.robotcore.external.matrices.VectorF
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.IRobotModule
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.collectors.events.IEvent
import org.firstinspires.ftc.teamcode.modules.camera.Camera.AddCameraProcessor
import org.firstinspires.ftc.teamcode.modules.intake.IntakeManager
import org.firstinspires.ftc.teamcode.modules.navigation.gyro.MergeGyro
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Vec2
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.lang.Math.toRadians
import kotlin.math.sqrt


class CVOdometry : IRobotModule {
    class UpdateCVOdometryEvent(val pos: Vec2) : IEvent

    private lateinit var _aprilTagProcessor: AprilTagProcessor

    private lateinit var _eventBus: EventBus

    override fun init(collector: BaseCollector, bus: EventBus){}

    override fun lateInit(collector: BaseCollector, bus: EventBus) {
        _eventBus = bus

        if (Configs.CVOdometryConfig.USE_CAMERA) {
            _aprilTagProcessor =
                AprilTagProcessor.Builder().setOutputUnits(DistanceUnit.CM, AngleUnit.RADIANS)
                    .setCameraPose(
                        Position(
                            DistanceUnit.CM,
                            0.0,
                            0.0,
                            0.0,
                            0),
                        YawPitchRollAngles(
                            AngleUnit.RADIANS,
                            toRadians(90.0),
                            toRadians(0.0),
                            toRadians(0.0),
                            0
                        ))
                    .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                    .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
                    .build()

            bus.invoke(AddCameraProcessor(_aprilTagProcessor))
        }
    }

    override fun update() {
        if (_eventBus.invoke(IntakeManager.RequestLiftPosEvent()).pos != IntakeManager.LiftPosition.TRANSPORT ||
            !Configs.CVOdometryConfig.USE_CAMERA ||
            !_eventBus.invoke(IntakeManager.RequestLiftAtTargetEvent()).target!!)
            return

        val detections = _aprilTagProcessor.detections

        StaticTelemetry.addData("detections size", detections.size)

        if (detections.isEmpty())
            return

        var posSum = Vec2.ZERO
        var normalDetections = 0

        for (i in detections) {
            if(i.rawPose == null)
                continue

            // Считать позицию тэга относительно камеры и записать её в VectorF
            val rawTagPose: AprilTagPoseRaw = i.rawPose
            var rawTagPoseVector = VectorF(
                rawTagPose.x.toFloat(), rawTagPose.y.toFloat(), rawTagPose.z.toFloat()
            )

            // Считать вращение тега относительно камеры
            val rawTagRotation = rawTagPose.R

            // Cчитать метаданные из тэга
            val metadata: AprilTagMetadata = i.metadata

            // Достать позицию тега относительно поля
            val fieldTagPos =
                metadata.fieldPosition.multiplied(DistanceUnit.mmPerInch.toFloat() / 10f)

            // Достать угол тега относительно поля
            val fieldTagQ = metadata.fieldOrientation

            // Повернуть вектор относительного положения на угол между камерой и тегом
            rawTagPoseVector = rawTagRotation.inverted().multiplied(rawTagPoseVector)

            // Повернуть относительное положение на угол между тегом и полем
            val rotatedPosVector = fieldTagQ.applyToVector(rawTagPoseVector)

            val dist = sqrt(rotatedPosVector.get(0) * rotatedPosVector.get(0) + rotatedPosVector.get(1) * rotatedPosVector.get(1))

            if(dist > Configs.CVOdometryConfig.DETECT_DIST)
                continue

            val fieldCameraPos = fieldTagPos.subtracted(rotatedPosVector)

            posSum += Vec2(
                fieldCameraPos.get(0).toDouble(),
                fieldCameraPos.get(1).toDouble()
            )

            normalDetections++
        }

        if(normalDetections == 0)
            return

        _eventBus.invoke(UpdateCVOdometryEvent((posSum / Vec2(normalDetections.toDouble())) -
                Configs.CVOdometryConfig.CAMERA_POSITION.turn(_eventBus.invoke(MergeGyro.RequestMergeGyroEvent()).rotation!!.angle)))
    }
}
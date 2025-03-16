package org.firstinspires.ftc.teamcode.modules.camera

import android.graphics.Bitmap
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.android.Utils
import org.opencv.core.Core.ROTATE_180
import org.opencv.core.Core.multiply
import org.opencv.core.Core.rotate
import org.opencv.core.Mat
import org.opencv.imgproc.Imgproc.blur
import org.firstinspires.ftc.robotcore.external.function.Continuation
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Color
import org.firstinspires.ftc.teamcode.utils.units.Orientation
import org.firstinspires.ftc.teamcode.utils.units.Vec2
import org.opencv.core.Core
import org.opencv.core.Core.add
import org.opencv.core.Core.bitwise_and
import org.opencv.core.Core.inRange
import org.opencv.core.Core.split
import org.opencv.core.Core.subtract
import org.opencv.core.CvType.CV_32F
import org.opencv.core.CvType.CV_8U
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
import org.opencv.core.RotatedRect
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE
import org.opencv.imgproc.Imgproc.MORPH_ERODE
import org.opencv.imgproc.Imgproc.RETR_TREE
import org.opencv.imgproc.Imgproc.dilate
import org.opencv.imgproc.Imgproc.erode
import org.opencv.imgproc.Imgproc.findContours
import org.opencv.imgproc.Imgproc.getStructuringElement
import org.opencv.imgproc.Imgproc.line
import org.opencv.imgproc.Imgproc.minAreaRect
import org.opencv.imgproc.Imgproc.putText
import org.opencv.imgproc.Imgproc.resize
import java.util.concurrent.atomic.AtomicReference
import kotlin.math.abs


class StickProcessor : VisionProcessor, CameraStreamSource {
    var allianceSticks = AtomicReference<Array<Orientation>>(arrayOf())
    var yellowSticks = AtomicReference<Array<Orientation>>(arrayOf())

    var gameColor = AtomicReference(BaseCollector.GameColor.BLUE)

    private var _isOneFrame = AtomicReference(false)

    private var lastFrame: AtomicReference<Bitmap> =
        AtomicReference(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565))

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {

    }

    private var _drawFrame = Mat()

    override fun processFrame(frm: Mat?, captureTimeNanos: Long): Any {
        val frame = frm!!.clone()

//        if (!_isOneFrame.get())
//            return frm

        resize(
            frame,
            frame,
            Size(
                frame.width() * Configs.CameraConfig.COMPRESSION_COEF,
                frame.height() * Configs.CameraConfig.COMPRESSION_COEF
            )
        )

        rotate(frame, frame, ROTATE_180)

        frame.copyTo(_drawFrame)

        blur(
            frame,
            frame,
            Size(Configs.CameraConfig.BLUR_SIZE, Configs.CameraConfig.BLUR_SIZE)
        )

        val colors = mutableListOf<Mat>()

        split(frame, colors)

        val r = colors[0]
        val g = colors[1]
        val b = colors[2]

        val yellowRects = detectElements(
            r, g, b,
            Configs.CameraConfig.YELLOW_STICK_DETECT_CONFIG.KR,
            Configs.CameraConfig.YELLOW_STICK_DETECT_CONFIG.KG,
            Configs.CameraConfig.YELLOW_STICK_DETECT_CONFIG.KB,
            Configs.CameraConfig.YELLOW_STICK_DETECT_CONFIG.THREASHOLD
        )

        val gameCol = gameColor.get()

        val allianceRects = detectElements(
            r, g, b,
            if (gameCol == BaseCollector.GameColor.BLUE)
                Configs.CameraConfig.BLUE_STICK_DETECT_CONFIG.KR
            else Configs.CameraConfig.RED_STICK_DETECT_CONFIG.KR,
            if (gameCol == BaseCollector.GameColor.BLUE)
                Configs.CameraConfig.BLUE_STICK_DETECT_CONFIG.KG
            else Configs.CameraConfig.RED_STICK_DETECT_CONFIG.KG,
            if (gameCol == BaseCollector.GameColor.BLUE)
                Configs.CameraConfig.BLUE_STICK_DETECT_CONFIG.KB
            else Configs.CameraConfig.RED_STICK_DETECT_CONFIG.KB,
            if (gameCol == BaseCollector.GameColor.BLUE)
                Configs.CameraConfig.BLUE_STICK_DETECT_CONFIG.THREASHOLD
            else Configs.CameraConfig.RED_STICK_DETECT_CONFIG.THREASHOLD
        )

        drawRotatedRects(
            _drawFrame, yellowRects,
            Configs.CameraConfig.YELLOW_STICK_DETECT_CONFIG.CONTOUR_COLOR,
            Configs.CameraConfig.YELLOW_STICK_DETECT_CONFIG.TEXT_COLOR
        )

        drawRotatedRects(
            _drawFrame, allianceRects,
            if (gameCol == BaseCollector.GameColor.BLUE)
                Configs.CameraConfig.BLUE_STICK_DETECT_CONFIG.CONTOUR_COLOR
            else
                Configs.CameraConfig.RED_STICK_DETECT_CONFIG.CONTOUR_COLOR,
            if (gameCol == BaseCollector.GameColor.BLUE)
                Configs.CameraConfig.BLUE_STICK_DETECT_CONFIG.TEXT_COLOR
            else
                Configs.CameraConfig.RED_STICK_DETECT_CONFIG.TEXT_COLOR
        )

        yellowSticks.set(rotatedRectToOrientation(yellowRects))
        allianceSticks.set(rotatedRectToOrientation(allianceRects))

        val bitmap = Bitmap.createBitmap(
            _drawFrame.width(),
            _drawFrame.height(),
            Bitmap.Config.RGB_565
        )
        Utils.matToBitmap(_drawFrame, bitmap)
        lastFrame.set(bitmap)

        _isOneFrame.set(false)

        return frm
    }

    fun waitFrame(){
        _isOneFrame.set(true)

        while(_isOneFrame.get());
    }

    override fun onDrawFrame(
        canvas: android.graphics.Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?
    ) {

    }

    fun rotatedRectToOrientation(rects: List<RotatedRect>): Array<Orientation> {
        val rectsList = rects.toList()

        return Array(rectsList.size) {
            val pos = rectsList[it].center

            Orientation(
                Vec2(pos.x, pos.y), Angle.ofDeg(
                    (if (rectsList[it].size.width < rectsList[it].size.height)
                        rectsList[it].angle
                    else
                        rectsList[it].angle - 90.0)
                )
            )
        }
    }

    private fun detectElements(
        r: Mat,
        g: Mat,
        b: Mat,
        kr: Double,
        kg: Double,
        kb: Double,
        threshold: Double
    ): List<RotatedRect> {
        val uR = Mat()
        val uG = Mat()
        val uB = Mat()

        val matOfOnes = Mat.ones(r.size(), CV_8U)

        multiply(r, matOfOnes, uR, kr)
        multiply(g, matOfOnes, uG, kg)
        multiply(b, matOfOnes, uB, kb)

        val combined = Mat()

        add(uR, uB, combined)
        add(combined, uG, combined)

        multiply(r, matOfOnes, uR, 1.0 - kr)
        multiply(g, matOfOnes, uG, 1.0 - kg)
        multiply(b, matOfOnes, uB, 1.0 - kb)

        subtract(combined, uR, uR)
        subtract(combined, uG, uG)
        subtract(combined, uB, uB)

        inRange(uR, Scalar(threshold), Scalar(255.0), uR)
        inRange(uG, Scalar(threshold), Scalar(255.0), uG)
        inRange(uB, Scalar(threshold), Scalar(255.0), uB)

        bitwise_and(uR, uG, combined)
        bitwise_and(combined, uB, combined)

        erodeDilate(combined, Configs.CameraConfig.ERODE_DILATE_K)

        val contours = mutableListOf<MatOfPoint>()

        findContours(combined, contours, Mat(), RETR_TREE, CHAIN_APPROX_SIMPLE)

        val points = MatOfPoint2f()


        return MutableList(contours.size) {
            points.fromArray(*contours[it].toArray())
            val rect = minAreaRect(points)

            val attitude =
                if (rect.size.width > rect.size.height)
                    rect.size.width / rect.size.height
                else
                    rect.size.height / rect.size.width

            if (abs(attitude - Configs.CameraConfig.STICK_ATTITUDE) < Configs.CameraConfig.STICK_ATTITUDE_SENS)
                rect
            else
                null
        }.filterNotNull()
    }

    fun erodeDilate(mat: Mat, kSize: Double) {
        erode(mat, mat, getStructuringElement(MORPH_ERODE, Size(kSize, kSize)))
        dilate(mat, mat, getStructuringElement(MORPH_ERODE, Size(kSize, kSize)))
    }

    fun drawRotatedRects(
        mat: Mat,
        rects: Collection<RotatedRect>,
        rectColor: Color,
        textColor: Color
    ) {
        val scalarRectColor =
            Scalar(rectColor.r.toDouble(), rectColor.g.toDouble(), rectColor.b.toDouble())

        for (i in rects) {
            val points = Array<Point?>(4) { null }

            i.points(points)

            line(mat, points[0], points[1], scalarRectColor, 5)
            line(mat, points[1], points[2], scalarRectColor, 5)
            line(mat, points[2], points[3], scalarRectColor, 5)
            line(mat, points[3], points[0], scalarRectColor, 5)

            putText(
                mat,
                ((if (i.size.width < i.size.height)
                    i.angle
                else
                    i.angle - 90.0)).toInt().toString(),
                i.center,
                5,
                2.0,
                Scalar(textColor.r.toDouble(), textColor.g.toDouble(), textColor.b.toDouble())
            )
        }
    }

    override fun getFrameBitmap(continuation: Continuation<out org.firstinspires.ftc.robotcore.external.function.Consumer<Bitmap?>?>?) {
        continuation!!.dispatch { bitmapConsumer -> bitmapConsumer?.accept(lastFrame.get()) }
    }
}
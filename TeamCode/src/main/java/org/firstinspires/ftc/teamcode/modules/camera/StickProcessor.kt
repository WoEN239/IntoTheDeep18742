package org.firstinspires.ftc.teamcode.modules.camera

import android.graphics.Bitmap
import android.graphics.Canvas
import org.firstinspires.ftc.robotcore.external.function.Consumer
import org.firstinspires.ftc.robotcore.external.function.Continuation
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.android.Utils
import org.opencv.core.Core.inRange
import org.opencv.core.Core.normalize
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
import org.opencv.core.RotatedRect
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE
import org.opencv.imgproc.Imgproc.COLOR_RGB2HSV
import org.opencv.imgproc.Imgproc.MORPH_ERODE
import org.opencv.imgproc.Imgproc.RETR_TREE
import org.opencv.imgproc.Imgproc.blur
import org.opencv.imgproc.Imgproc.cvtColor
import org.opencv.imgproc.Imgproc.dilate
import org.opencv.imgproc.Imgproc.erode
import org.opencv.imgproc.Imgproc.findContours
import org.opencv.imgproc.Imgproc.getStructuringElement
import org.opencv.imgproc.Imgproc.line
import org.opencv.imgproc.Imgproc.minAreaRect
import org.opencv.imgproc.Imgproc.putText
import java.util.concurrent.Callable
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicReference


class StickProcessor : VisionProcessor, CameraStreamSource {
    private var lastFrame: AtomicReference<Bitmap> =
        AtomicReference(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565))

    private val _executorService: ExecutorService =
        Executors.newWorkStealingPool(Configs.CameraConfig.DETECT_THREADS_COUNT)

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {

    }

    override fun processFrame(frame: Mat, captureTimeNanos: Long): Any {
        val cloneFrame = frame.clone()

        normalize(cloneFrame, cloneFrame, 100.0)

        val drawFrame = frame.clone()
        val hsvFrame = frame.clone()

        blur(hsvFrame, hsvFrame, Size(10.0, 10.0))
        cvtColor(hsvFrame, hsvFrame, COLOR_RGB2HSV)

        val blueRects = detect(Configs.CameraConfig.BLUE_STICK_DETECT, hsvFrame.clone())

        drawRotatedRects(drawFrame, blueRects, Scalar(0.0, 0.0, 255.0), Scalar(0.0, 255.0, 0.0))

        val redRects = detect(Configs.CameraConfig.RED_STICK_DETECT, hsvFrame.clone())

        drawRotatedRects(drawFrame, redRects, Scalar(255.0, 0.0, 0.0), Scalar(0.0, 255.0, 0.0))

        val b = Bitmap.createBitmap(
            drawFrame.width(),
            drawFrame.height(),
            Bitmap.Config.RGB_565
        )
        Utils.matToBitmap(drawFrame, b)
        lastFrame.set(b)

        return frame
    }

    private fun detect(
        parameters: Configs.CameraConfig.StickDetectConfig,
        hsvFrame: Mat
    ): Collection<RotatedRect> {
        inRange(
            hsvFrame,
            Scalar(parameters.H_MIN, parameters.S_MIN, parameters.V_MIN),
            Scalar(parameters.H_MAX, parameters.S_MAX, parameters.V_MAX),
            hsvFrame
        )

        erodeDilate(hsvFrame, parameters.ERODE_DILATE)

        erode(
            hsvFrame,
            hsvFrame,
            getStructuringElement(
                MORPH_ERODE,
                Size(parameters.PRECOMPRESSION, parameters.PRECOMPRESSION)
            )
        )

        dilateErode(hsvFrame, parameters.DILATE_ERODE)

        dilate(
            hsvFrame,
            hsvFrame,
            getStructuringElement(
                MORPH_ERODE,
                Size(parameters.PRECOMPRESSION, parameters.PRECOMPRESSION)
            )
        )

        val contours = arrayListOf<MatOfPoint>()

        findContours(hsvFrame, contours, Mat(), RETR_TREE, CHAIN_APPROX_SIMPLE)

        val tasks = arrayListOf<Callable<RotatedRect?>>()

        for (i in contours)
            tasks.add { processContour(i) }

        val result = _executorService.invokeAll(tasks)

        return result.mapNotNull { it.get() }
    }

    fun drawRotatedRects(mat: Mat, rects: Collection<RotatedRect>, rectColor: Scalar, textColor: Scalar) {
        for (i in rects) {
            val points = Array<Point?>(4) { null }

            i.points(points)

            line(mat, points[0], points[1], rectColor, 5)
            line(mat, points[1], points[2], rectColor, 5)
            line(mat, points[2], points[3], rectColor, 5)
            line(mat, points[3], points[0], rectColor, 5)

            putText(
                mat,
                i.angle.toInt().toString(),
                i.center,
                5,
                2.0,
                textColor
            )
        }
    }

    private fun processContour(contour: MatOfPoint): RotatedRect? {
        val points = MatOfPoint2f()

        points.fromArray(*contour.toArray())

        val rect = minAreaRect(points)

        if (rect.size.height * rect.size.width > Configs.CameraConfig.MIN_STICK_AREA)
            return rect

        return null
    }

    override fun onDrawFrame(
        canvas: Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?
    ) {

    }

    fun erodeDilate(mat: Mat, kSize: Double) {
        erode(mat, mat, getStructuringElement(MORPH_ERODE, Size(kSize, kSize)))
        dilate(mat, mat, getStructuringElement(MORPH_ERODE, Size(kSize, kSize)))
    }

    fun dilateErode(mat: Mat, kSize: Double) {
        dilate(mat, mat, getStructuringElement(MORPH_ERODE, Size(kSize, kSize)))
        erode(mat, mat, getStructuringElement(MORPH_ERODE, Size(kSize, kSize)))
    }

    override fun getFrameBitmap(continuation: Continuation<out Consumer<Bitmap>>?) {
        continuation!!.dispatch { bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()) }
    }
}
package org.firstinspires.ftc.teamcode.modules.camera

import android.graphics.Bitmap
import android.graphics.Canvas
import android.os.Environment
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
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgcodecs.Imgcodecs.imread
import org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE
import org.opencv.imgproc.Imgproc.COLOR_BGR2RGB
import org.opencv.imgproc.Imgproc.COLOR_RGB2HSV
import org.opencv.imgproc.Imgproc.MORPH_ERODE
import org.opencv.imgproc.Imgproc.RETR_TREE
import org.opencv.imgproc.Imgproc.arrowedLine
import org.opencv.imgproc.Imgproc.blur
import org.opencv.imgproc.Imgproc.cvtColor
import org.opencv.imgproc.Imgproc.dilate
import org.opencv.imgproc.Imgproc.erode
import org.opencv.imgproc.Imgproc.findContours
import org.opencv.imgproc.Imgproc.getStructuringElement
import org.opencv.imgproc.Imgproc.line
import org.opencv.imgproc.Imgproc.minAreaRect
import org.opencv.imgproc.Imgproc.putText
import org.opencv.imgproc.Imgproc.rectangle
import org.opencv.imgproc.Imgproc.resize
import java.lang.System.out
import java.util.concurrent.atomic.AtomicReference


class StickProcessor : VisionProcessor, CameraStreamSource {
    private var lastFrame: AtomicReference<Bitmap> =
        AtomicReference(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565))

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {

    }

    override fun processFrame(frame: Mat, captureTimeNanos: Long): Any {
        val cloneFrame = frame.clone()

        normalize(cloneFrame, cloneFrame, 100.0)

        val drawFrame = frame.clone()
        val hsvFrame = frame.clone()

        blur(hsvFrame, hsvFrame, Size(10.0, 10.0))
        cvtColor(hsvFrame, hsvFrame, COLOR_RGB2HSV)

        val blueBinaryFrame = hsvFrame.clone()

        inRange(
            blueBinaryFrame,
            Scalar(
                Configs.CameraConfig.BLUE_H_MIN,
                Configs.CameraConfig.BLUE_S_MIN,
                Configs.CameraConfig.BLUE_V_MIN
            ),
            Scalar(
                Configs.CameraConfig.BLUE_H_MAX,
                Configs.CameraConfig.BLUE_S_MAX,
                Configs.CameraConfig.BLUE_V_MAX
            ),
            blueBinaryFrame
        )

        erodeDilate(blueBinaryFrame, Configs.CameraConfig.ERODE_DILATE_BLUE)

        erode(blueBinaryFrame, blueBinaryFrame, getStructuringElement(MORPH_ERODE, Size(Configs.CameraConfig.PRECOMPRESSION_BLUE, Configs.CameraConfig.PRECOMPRESSION_BLUE)))

        dilateErode(blueBinaryFrame, Configs.CameraConfig.DILATE_ERODE_BLUE)

        dilate(blueBinaryFrame, blueBinaryFrame, getStructuringElement(MORPH_ERODE, Size(Configs.CameraConfig.PRECOMPRESSION_BLUE, Configs.CameraConfig.PRECOMPRESSION_BLUE)))

        val contours = arrayListOf<MatOfPoint>()

        findContours(blueBinaryFrame, contours, Mat(), RETR_TREE, CHAIN_APPROX_SIMPLE)

        for(i in contours){
            val points = MatOfPoint2f()

            points.fromArray(*i.toArray())

            val rect = minAreaRect(points)



            if(rect.size.height * rect.size.width > Configs.CameraConfig.MIN_STICK_AREA) {
                //rectangle(drawFrame, rect.boundingRect(), Scalar(0.0, 0.0, 255.0), 5)

                val points = Array<Point?>(4){null}

                rect.points(points)

                line(drawFrame, points[0], points[1], Scalar(0.0, 0.0, 255.0), 5)
                line(drawFrame, points[1], points[2], Scalar(0.0, 0.0, 255.0), 5)
                line(drawFrame, points[2], points[3], Scalar(0.0, 0.0, 255.0), 5)
                line(drawFrame, points[3], points[0], Scalar(0.0, 0.0, 255.0), 5)

                putText(drawFrame, rect.angle.toInt().toString(), rect.center, 5, 2.0, Scalar(0.0, 255.0, 0.0))
            }
        }

        val b = Bitmap.createBitmap(
            drawFrame.width(),
            drawFrame.height(),
            Bitmap.Config.RGB_565
        )
        Utils.matToBitmap(drawFrame, b)
        lastFrame.set(b)

        return frame
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
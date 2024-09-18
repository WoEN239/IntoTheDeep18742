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
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgcodecs.Imgcodecs.imread
import org.opencv.imgproc.Imgproc.COLOR_BGR2RGB
import org.opencv.imgproc.Imgproc.COLOR_RGB2HSV
import org.opencv.imgproc.Imgproc.MORPH_ERODE
import org.opencv.imgproc.Imgproc.blur
import org.opencv.imgproc.Imgproc.cvtColor
import org.opencv.imgproc.Imgproc.dilate
import org.opencv.imgproc.Imgproc.erode
import org.opencv.imgproc.Imgproc.getStructuringElement
import org.opencv.imgproc.Imgproc.resize
import java.util.concurrent.atomic.AtomicReference


class StickProcessor : VisionProcessor, CameraStreamSource {
    private var lastFrame: AtomicReference<Bitmap> =
        AtomicReference(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565))

    private var _blueStickMat = AtomicReference<Mat>()

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {
        _blueStickMat.set(imread(Environment.getExternalStorageDirectory().toString() + "/" + Environment.DIRECTORY_DCIM + "/blue_stick.jpg"))
        cvtColor(_blueStickMat.get(), _blueStickMat.get(), COLOR_BGR2RGB)

        resize(
            _blueStickMat.get(),
            _blueStickMat.get(),
            Size(
                _blueStickMat.get().width() * Configs.CameraConfig.COMPRESSION_COEF,
                _blueStickMat.get().height() * Configs.CameraConfig.COMPRESSION_COEF
            )
        )
    }

    override fun processFrame(frame: Mat, captureTimeNanos: Long): Any {
        val cloneFrame = frame.clone()

        normalize(cloneFrame, cloneFrame, 100.0)

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

        k(blueBinaryFrame, Configs.CameraConfig.K_SIZE_BLUE)

        val b = Bitmap.createBitmap(
            blueBinaryFrame.width(),
            blueBinaryFrame.height(),
            Bitmap.Config.RGB_565
        )
        Utils.matToBitmap(blueBinaryFrame, b)
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

    fun k(mat: Mat, kSize: Double) {
        erode(mat, mat, getStructuringElement(MORPH_ERODE, Size(kSize, kSize)))
        dilate(mat, mat, getStructuringElement(MORPH_ERODE, Size(kSize, kSize)))
    }

    override fun getFrameBitmap(continuation: Continuation<out Consumer<Bitmap>>?) {
        continuation!!.dispatch { bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()) }
    }
}
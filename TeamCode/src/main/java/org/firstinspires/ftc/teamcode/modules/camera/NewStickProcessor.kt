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
import org.opencv.core.Core.ROTATE_180
import org.opencv.core.Core.inRange
import org.opencv.core.Core.rotate
import org.opencv.core.Mat
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc.COLOR_RGB2HSV
import org.opencv.imgproc.Imgproc.blur
import org.opencv.imgproc.Imgproc.cvtColor
import org.opencv.imgproc.Imgproc.moments
import org.opencv.imgproc.Imgproc.resize
import java.util.concurrent.atomic.AtomicReference
import kotlin.math.max

class NewStickProcessor : VisionProcessor, CameraStreamSource {
    private var lastFrame: AtomicReference<Bitmap> =
        AtomicReference(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565))

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {

    }

    private var _drawFrame = Mat()

    override fun processFrame(frm: Mat?, captureTimeNanos: Long): Any {
        val frame = frm!!.clone()

        resize(
            frame,
            frame,
            Size(
                frame.width() * Configs.CameraConfig.COMPRESSION_COEF,
                frame.height() * Configs.CameraConfig.COMPRESSION_COEF
            )
        )

        rotate(frame, frame, ROTATE_180)

        blur(frame, frame, Size(5.0, 5.0))
//
//        val hsvFrame = Mat()
//
//        cvtColor(frame, hsvFrame, COLOR_RGB2HSV)

//        inRange(
//            hsvFrame,
//            Scalar(Configs.CameraConfig.YELLOW_STICK_DETECT.H_MIN, Configs.CameraConfig.YELLOW_STICK_DETECT.S_MIN, Configs.CameraConfig.YELLOW_STICK_DETECT.V_MIN),
//            Scalar(Configs.CameraConfig.YELLOW_STICK_DETECT.H_MAX, Configs.CameraConfig.YELLOW_STICK_DETECT.S_MAX, Configs.CameraConfig.YELLOW_STICK_DETECT.V_MAX),
//            hsvFrame
//        )
//
//        mo

        val binaryFrame = Mat(frame.rows(), frame.cols(), frame.type())

        for(row in 0..<frame.rows())
            for(colum in 0..<frame.cols()){
                val colors = frame.get(row, colum)

                val r = colors[0]
                val g = colors[1]
                val b = colors[2]

                val bColors = binaryFrame.get(row, colum)

                if(r - max(g, b) > Configs.CameraConfig2.R_TREASHOLD) {
                    for(i in bColors.indices)
                        bColors[i] = 255.0
                }
                else
                    for(i in bColors.indices)
                        bColors[i] = 0.0
            }

        _drawFrame = frame

        val b = Bitmap.createBitmap(
            _drawFrame.width(),
            _drawFrame.height(),
            Bitmap.Config.RGB_565
        )
        Utils.matToBitmap(_drawFrame, b)
        lastFrame.set(b)


        return frm
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

    override fun getFrameBitmap(continuation: Continuation<out Consumer<Bitmap>>?) {
        continuation!!.dispatch { bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()) }
    }
}
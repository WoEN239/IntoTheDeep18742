package org.firstinspires.ftc.teamcode.utils.telemetry

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.utils.configs.Configs
import org.firstinspires.ftc.teamcode.utils.units.Color
import org.firstinspires.ftc.teamcode.utils.units.Vec2

/**
 * Синглтон для телеметрии
 * обьединяет телеметрию с дашборда и телефона
 *
 * @author tikhonsmovzh
 */
object StaticTelemetry {
    private lateinit var _phoneTelemetry: Telemetry
    private var _telemetryPacket = TelemetryPacket()

    fun setPhoneTelemetry(telemetry: Telemetry) {
        _phoneTelemetry = telemetry
    }

    fun addLine(str: String) {
        _telemetryPacket.addLine(str)
        _phoneTelemetry.addLine(str)
    }

    fun addData(name: String, obj: Any) {
        _phoneTelemetry.addData(name, obj)
        _telemetryPacket.put(name, obj)
    }

    private var _deltaTime = ElapsedTime()

    fun update() {
        if(Configs.TelemetryConfig.ENABLE && _deltaTime.seconds() > 1.0 / Configs.TelemetryConfig.SEND_HZ) {
            _deltaTime.reset()
            _phoneTelemetry.update()

            FtcDashboard.getInstance().sendTelemetryPacket(_telemetryPacket)
        }

        _phoneTelemetry.clearAll()

        _telemetryPacket = TelemetryPacket()
    }

    var canvas: Canvas
        get() = _telemetryPacket.fieldOverlay()
        private set(v) {}

    fun drawCircle(pos: Vec2, radius: Double, color: String) {
        canvas.setFill(color)
        canvas.fillCircle(
            DistanceUnit.INCH.fromCm(pos.x),
            DistanceUnit.INCH.fromCm(pos.y),
            DistanceUnit.INCH.fromCm(radius)
        )
    }

    fun drawCircle(pos: Vec2, radius: Double, color: Color) =
        drawCircle(pos, radius, color.toString())

    fun drawPolygon(points: Array<Vec2>, color: String) {
        val inchX = DoubleArray(points.size)
        val inchY = DoubleArray(points.size)

        for (i in points.indices) {
            inchX[i] = DistanceUnit.INCH.fromCm(points[i].x)
            inchY[i] = DistanceUnit.INCH.fromCm(points[i].y)
        }

        canvas.setFill(color)
        canvas.fillPolygon(inchX, inchY)
    }

    fun drawPolygon(points: Array<Vec2>, color: Color) = drawPolygon(points, color.toString())

    fun drawRect(center: Vec2, size: Vec2, rot: Double = 0.0, color: String) =
        drawPolygon(
            arrayOf(
                center + Vec2(-size.x / 2, size.y / 2).turn(rot),
                center + Vec2(size.x / 2, size.y / 2).turn(rot),
                center + Vec2(size.x / 2, -size.y / 2).turn(rot),
                center + Vec2(-size.x / 2, -size.y / 2).turn(rot)
            ), color
        )

    fun drawRect(center: Vec2, size: Vec2, rot: Double = 0.0, color: Color) =
        drawRect(center, size, rot, color.toString())
}
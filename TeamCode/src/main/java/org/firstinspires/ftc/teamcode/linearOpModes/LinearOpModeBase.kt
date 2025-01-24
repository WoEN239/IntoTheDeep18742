package org.firstinspires.ftc.teamcode.linearOpModes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.utils.telemetry.StaticTelemetry
import org.firstinspires.ftc.teamcode.utils.units.Angle
import org.firstinspires.ftc.teamcode.utils.units.Vec2

/**
 * Класс для всех опмодов который запускает всю программу
 *
 * @author tikhonsmovzh
 */
open class LinearOpModeBase : LinearOpMode() {
    data class OpModeSettings(
        val isAutoStart: Boolean,
        val isPreInit: Boolean,
        val preInitOpModeName: String = "",
        val initTime: Double = 1.5
    )

    protected open fun getOpModeSettings() = OpModeSettings(isAutoStart = false, isPreInit = false)

    protected open fun getCollector() = BaseCollector(this, BaseCollector.GameSettings(startPosition = BaseCollector.GameStartPosition.NONE), isAuto = false)

    override fun runOpMode() {
        StaticTelemetry.setPhoneTelemetry(telemetry)

        try {
            val settings = getOpModeSettings()

            val collector = getCollector()

            collector.init()

            if(settings.isAutoStart)
                OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getActivity()).startActiveOpMode()

            waitForStart()
            resetRuntime()

            collector.start()

            while (opModeIsActive()) {
                StaticTelemetry.update()

                collector.update()
            }

            collector.stop()

            if(settings.isPreInit)
                OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getActivity()).initOpMode(settings.preInitOpModeName)
        } catch (e: Exception) {
            throw e
        }
    }
}
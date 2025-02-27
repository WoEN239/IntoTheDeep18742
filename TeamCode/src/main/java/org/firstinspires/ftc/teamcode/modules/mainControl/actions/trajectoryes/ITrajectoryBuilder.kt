package org.firstinspires.ftc.teamcode.modules.mainControl.actions.trajectoryes

import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.utils.units.Orientation

interface ITrajectoryBuilder {
    fun runTrajectory(eventBus: EventBus, startOrientation: Orientation)
}
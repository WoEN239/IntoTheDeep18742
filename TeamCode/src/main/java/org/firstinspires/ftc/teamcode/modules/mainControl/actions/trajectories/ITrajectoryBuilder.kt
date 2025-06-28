package org.firstinspires.ftc.teamcode.modules.mainControl.actions.trajectories

import org.firstinspires.ftc.teamcode.collectors.BaseCollector
import org.firstinspires.ftc.teamcode.collectors.events.EventBus
import org.firstinspires.ftc.teamcode.utils.units.Orientation

interface ITrajectoryBuilder {
    fun runTrajectory(
        eventBus: EventBus,
        startOrientation: Orientation,
        teammate: BaseCollector.TeammateSate
    )
}
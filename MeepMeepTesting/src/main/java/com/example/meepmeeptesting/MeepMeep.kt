import com.acmerobotics.roadrunner.Pose2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder

object MeepMeepTesting {
    @JvmStatic
    fun main(args: Array<String>) {
        val meepMeep = MeepMeep(800)

        val myBot =
            DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 11.81)
                .build()

        myBot.runAction(
            myBot.drive.actionBuilder(Pose2d(0.0, 0.0, 0.0))
                .lineToX(30.0)
                .turn(Math.toRadians(90.0))
                .lineToY(30.0)
                .turn(Math.toRadians(90.0))
                .lineToX(0.0)
                .turn(Math.toRadians(90.0))
                .lineToY(0.0)
                .turn(Math.toRadians(90.0))
                .build()
        )

        meepMeep.setBackground(Background.FIELD_INTO_THE_DEEP_OFFICIAL)
            .setDarkMode(false)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start()
    }
}
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
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
            myBot.drive.actionBuilder(Pose2d(-16.0, 61.0, Math.toRadians(-90.0)))
                .waitSeconds(5.0)
                .splineTo(Vector2d(-5.0,35.0),Math.toRadians(-90.0))
                .build()
        )

        meepMeep.setBackground(Background.FIELD_INTO_THE_DEEP_OFFICIAL)
            .setDarkMode(false)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start()
    }
}
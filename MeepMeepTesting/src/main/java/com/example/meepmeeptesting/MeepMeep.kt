import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.core.util.FieldUtil
import com.noahbres.meepmeep.roadrunner.Constraints
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.DriveTrainType
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity

object MeepMeepTesting {
    @JvmStatic
    fun main(args: Array<String>) {
        FieldUtil.Companion.FIELD_HEIGHT = 366
        FieldUtil.Companion.FIELD_WIDTH =  366

        val meepMeep = MeepMeep(600);
        meepMeep.setAxesInterval(1000);

        val myBot = RoadRunnerBotEntity(meepMeep,
             Constraints(130.0, 70.0, 7.9, 4.8, 30.0),
            38.8, 41.5,
         Pose2d(-1650.0, 1710.0, 0.0),
        meepMeep.colorManager.theme, 1.0, DriveTrainType.MECANUM, false
        );

        myBot.runAction(
myBot.drive.actionBuilder(Pose2d(42.0, -165.0, Math.toRadians(90.0)))
        .strafeTo(Vector2d(20.0, -85.0))
    .waitSeconds(5.0)
    .strafeTo(Vector2d(150.0,-159.0))
            .build())



        meepMeep.setBackground(Background.FIELD_INTO_THE_DEEP_OFFICIAL)
            .setDarkMode(false)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start()
    }
}
package team13580;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Red_Meep_meep_13580 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(17.7, 17.8)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-50, 50, Math.toRadians(126)))
                .lineToY(24)

                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-28,54, Math.toRadians(0)), Math.toRadians(90))

                .setTangent(Math.toRadians(0))
                .splineTo(new Vector2d(-6, 36),Math.toRadians(-90))
                .splineTo(new Vector2d(-30, 24), Math.toRadians(126))
                .waitSeconds(2)

                .turn(Math.toRadians(-36))
                .splineToConstantHeading(new Vector2d(16, 34),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(4, 55),Math.toRadians(90))
                .waitSeconds(.5)

                .lineToY(50)
                .splineToLinearHeading(new Pose2d(-30,24, Math.toRadians(126)), Math.toRadians(90))
                .waitSeconds(2)

                .turn(Math.toRadians(-36))
                .splineToConstantHeading(new Vector2d(38, 34),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(30, 56),Math.toRadians(90))

.setTangent(Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(-30,24, Math.toRadians(126)), Math.toRadians(180))
                .waitSeconds(1)

                .splineToLinearHeading(new Pose2d(-30,50, Math.toRadians(90)), Math.toRadians(90))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

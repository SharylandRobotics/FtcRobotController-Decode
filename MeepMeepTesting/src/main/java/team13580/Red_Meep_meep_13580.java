package team13580;

import com.acmerobotics.roadrunner.*;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.lang.Math;

public class Red_Meep_meep_13580 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(17.7, 17.8)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-50, 50, Math.toRadians(126)))
                //path1
                .lineToY(24)
                //path2
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-28,54, Math.toRadians(0)), Math.toRadians(90))
                //path3
                .setTangent(Math.toRadians(0))
                .splineTo(new Vector2d(-6, 36),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-2, 53),Math.toRadians(-90))
                .setTangent(Math.atan2(24-53, -30+2))
                .lineToYLinearHeading(24, Math.toRadians(126))
                //.splineTo(new Vector2d(-30, 24), Math.toRadians(126))
                .waitSeconds(2)
                //path4intake_middle
                .turn(Math.toRadians(-36))
                .splineToConstantHeading(new Vector2d(16, 34),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(10, 52),Math.toRadians(90))
                //path5
                .setTangent(Math.atan2(24-52, -30-10))
                        .lineToYLinearHeading(24, Math.toRadians(126))
                .waitSeconds(2)

                //path6_intakelast
                //.turn(Math.toRadians(-36))
                .setTangent(Math.atan2(-30+24, -38-30))
                .lineToYLinearHeading(30, Math.toRadians(90))
                //.splineToConstantHeading(new Vector2d(38, 34),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(30, 56),Math.toRadians(90))

                //path7_lastshot
                .setTangent(Math.atan2(24-56, -30-30))
                .lineToYLinearHeading(24, Math.toRadians(126), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                .waitSeconds(1)

                //path8_park
                .setTangent(Math.atan2(24-50, 0))
                .lineToYLinearHeading(54, Math.toRadians(90))
                //.splineToLinearHeading(new Pose2d(-30,50, Math.toRadians(90)), Math.toRadians(90))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

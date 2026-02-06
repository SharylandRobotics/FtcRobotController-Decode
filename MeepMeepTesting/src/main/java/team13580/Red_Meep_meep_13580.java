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
                .lineToYLinearHeading(24, Math.toRadians(126))
                //path2
                                .waitSeconds(2)
                //.setTangent(Math.toRadians(90))
                //path 2intake
                .setTangent(Math.atan2(54-24, -24+30))
                .lineToYLinearHeading(54, Math.toRadians(0))
                .setTangent(0)
                .lineToXLinearHeading(-22, Math.toRadians(-6))

                //path3firstintakeshot

                .setTangent(Math.atan2(36-54, -8+22))
                .lineToYLinearHeading(36, Math.toRadians(-90))
                //.splineTo(new Vector2d(-4, 36),Math.toRadians(-90))

                .setTangent(Math.atan2(54-36, -2+8))
                .lineToYLinearHeading(54, Math.toRadians(-90))
                //.splineToConstantHeading(new Vector2d(-2, 53),Math.toRadians(90))

                .setTangent(Math.atan2(24-54, -30+2))
                .lineToYLinearHeading(24, Math.toRadians(126), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                //.splineTo(new Vector2d(-30, 24), Math.toRadians(126))
                .waitSeconds(2)
                //path4intake_middle
                //.turn(Math.toRadians(-36))


               .setTangent(Math.atan2(30-24, 8+30))
                .lineToYLinearHeading(30, Math.toRadians(90))
                .setTangent(Math.atan2(56-30, 20-8))
                .lineToYLinearHeading(56, Math.toRadians(90))
                //path5middle shot
                .setTangent(Math.atan2(24-56, -30-20))
                .lineToYLinearHeading(24, Math.toRadians(126), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                .waitSeconds(2)

                //path6_intakelast
                //.turn(Math.toRadians(-36))
                .setTangent(Math.atan2(30-24, 35+30))
                .lineToYLinearHeading(30, Math.toRadians(90))
                .setTangent(Math.atan2(32-30, 35-35))
                .lineToYLinearHeading(33, Math.toRadians(90))
                .setTangent(Math.atan2(56-33, 40-35))
                .lineToYLinearHeading(56, Math.toRadians(80))

                //path7_lastshot
              .setTangent(Math.atan2(24-56, -30-40))
             .lineToYLinearHeading(24, Math.toRadians(126), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                .waitSeconds(1)

                //path8_park
               .setTangent(Math.atan2(24-50, 0))
                .lineToYLinearHeading(54, Math.toRadians(90), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

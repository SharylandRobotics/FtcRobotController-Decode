package team13580;

import com.acmerobotics.roadrunner.*;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.lang.Math;

public class help {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(17.7, 17.8)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-49, 51, Math.toRadians(126)))
                //path1
                .setTangent(Math.atan2(24-51, -30+49))
                .lineToY(24, new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                .waitSeconds(1.5)
                //path2
                .setTangent(Math.atan2(20-24, 12+30))
                .lineToYLinearHeading(20, Math.toRadians(90))
                .setTangent(Math.atan2(53-20, 0))
                .lineToYLinearHeading(53, Math.toRadians(90), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-60, 100))
                .setTangent(Math.atan2(20-53, 0))
                .lineToYLinearHeading(20, Math.toRadians(90), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-60, 100))
                //path 3
                .setTangent(Math.atan2(24-20, -16-12))
                .lineToYLinearHeading(24, Math.toRadians(140), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-60, 100))
                .waitSeconds(1.5)
                //path 4  First gate intake
                .setTangent(Math.atan2(20-24, 12+16))
                .lineToYLinearHeading(20, Math.toRadians(125))
                .setTangent(Math.atan2(50-20, 0))
                .lineToYLinearHeading(50, Math.toRadians(125), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-60, 100))
                .setTangent(Math.atan2(0, 15-12))
                .lineToX(15, new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                .waitSeconds(1)
                //path5
                .setTangent(Math.atan2(20-46, 12-15))
                .lineToYLinearHeading(20, Math.toRadians(125), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                .setTangent(Math.atan2(24-20, -16-12))
                .lineToYLinearHeading(24, Math.toRadians(140), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-60, 100))
                .waitSeconds(1.5)
                //path 6 second gate intake
                //path 8
                .setTangent(Math.atan2(18-24, -14+16))
                .lineToYLinearHeading(18, Math.toRadians(90))
                .setTangent(Math.atan2(48-18, 0))
                .lineToYLinearHeading(48, Math.toRadians(90), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-60, 100))
                //path 9
                .setTangent(Math.atan2(24-48, -30+14))
                .lineToYLinearHeading(24, Math.toRadians(126), new TranslationalVelConstraint(90), new ProfileAccelConstraint(-60, 100))
                .waitSeconds(1)
                //path 10 park
                .setTangent(Math.atan2(50-24, -20+30))
                .lineToYLinearHeading(50, Math.toRadians(90), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-60, 100))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

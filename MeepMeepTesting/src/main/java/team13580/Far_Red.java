package team13580;

import com.acmerobotics.roadrunner.*;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.lang.Math;

public class Far_Red {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(17.7, 17.8)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(62, 15, Math.toRadians(180)))
                //path1
                .setTangent(Math.atan2(0, 50-62))
                .lineToXLinearHeading(50,Math.toRadians(170),new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                        .waitSeconds(2)
                //path2
                .setTangent(Math.atan2(24-15, 36-50))
                .lineToYLinearHeading(24,Math.toRadians(90),new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                .setTangent(Math.atan2(60-24, 0))
                .lineToYLinearHeading(60,Math.toRadians(90),new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                //path3
                .setTangent(Math.atan2(15-60, 50-36))
                .lineToYLinearHeading(15,Math.toRadians(170),new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                                .waitSeconds(1.5)
                //path4
                .setTangent(Math.atan2(60-15, 62-50))
                .lineToYLinearHeading(60,Math.toRadians(90),new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                .setTangent(Math.atan2(15-60, 50-62))
                .lineToYLinearHeading(15,Math.toRadians(170),new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                                .waitSeconds(1.5)

                //path 5

                .setTangent(Math.atan2(60-15, 0))
                .lineToYLinearHeading(60,Math.toRadians(90),new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                .setTangent(Math.atan2(15-60, 0))
                .lineToYLinearHeading(15,Math.toRadians(170),new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                        .waitSeconds(1)
                //path 6
                .setTangent(Math.atan2(60-15, 0))
                .lineToYLinearHeading(60,Math.toRadians(90),new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                .setTangent(Math.atan2(15-60, 0))
                .lineToYLinearHeading(15,Math.toRadians(170),new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                .waitSeconds(1)

                //path 7
                .setTangent(Math.atan2(60-15, 0))
                .lineToYLinearHeading(60,Math.toRadians(90),new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                .setTangent(Math.atan2(15-60, 0))
                .lineToYLinearHeading(15,Math.toRadians(170),new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                .waitSeconds(1)
                //path 8
                .setTangent(Math.atan2(60-15, 0))
                .lineToYLinearHeading(60,Math.toRadians(90),new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                .setTangent(Math.atan2(15-60, 0))
                .lineToYLinearHeading(15,Math.toRadians(170),new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                .waitSeconds(1)
                //Park
                .setTangent(Math.atan2(40-15, 40-50))
                .lineToYLinearHeading(40,Math.toRadians(90),new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))








                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

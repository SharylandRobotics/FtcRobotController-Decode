package team12395;

import com.acmerobotics.roadrunner.*;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.lang.Math;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

                myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-38, -54, Math.toRadians(-90)))
                                .setTangent(Math.atan2(-23 + 54, -23 + 38))
                                .lineToY(-23) // shoot  & then scan pattern

                                .setTangent(Math.toRadians(-20))
                                .splineToConstantHeading(new Vector2d(-11.5, -30), Math.toRadians(-90)) // move to row 1

                                //.setTangent(Math.toRadians(-90))
                                .lineToY(-48) // pick up balls

                                //.setTangent(Math.toRadians(-90))
                                .splineTo(new Vector2d(-12, -20), Math.toRadians(90))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
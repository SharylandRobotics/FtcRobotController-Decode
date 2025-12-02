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

                myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-40, -54, Math.toRadians(-90)))
                                .setTangent(Math.atan2(-23 + 54, -23 + 40))
                                .lineToY(-23) // shoot  & then scan pattern

                                .setTangent(Math.toRadians(-20))
                                .splineToConstantHeading(new Vector2d(-11.5, -30), Math.toRadians(-90)) // move to row 1

                                //.setTangent(Math.toRadians(-90))
                                .lineToY(-43) // pick up balls & double back (48)
                                .splineToSplineHeading(new Pose2d(-3, -52, Math.toRadians(-90)), Math.toRadians(60))
                                .splineToConstantHeading(new Vector2d(-10, -22), Math.toRadians(90))
                                //shoot

                                .setTangent(Math.atan2(-35 + 22, 11.5 + 10))
                                .splineToConstantHeading(new Vector2d(11.5, -35), Math.toRadians(-90))
                                .lineToY(-43)
                                .splineToSplineHeading(new Pose2d(-10, -22, Math.toRadians(-90)), Math.toRadians(135))
                                //shoot

                                .setTangent(Math.atan2(-38 + 22, 35 + 10))
                                .splineToConstantHeading(new Vector2d(35, -38), Math.toRadians(-90))
                                .lineToY(-43)
                                .splineToSplineHeading(new Pose2d(-10, -22, Math.toRadians(-90)), Math.toRadians(135))
                                //shoot

                                .lineToX(0)

                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
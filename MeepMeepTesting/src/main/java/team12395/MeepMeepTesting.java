package team12395;

import com.acmerobotics.roadrunner.*;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.lang.Math;
import java.time.temporal.Temporal;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        double cameraOffset = 5;
        double turretAngle = -45;

        Pose2d example = new Pose2d(0, 0, Math.toRadians(45));



        // camera-turret axis offset
        example = example.plus(new Twist2d(new Vector2d(-cameraOffset,0), Math.toRadians(0)));


        // turret rotation
        example = example.plus(new Twist2d(new Vector2d(0,0), Math.toRadians(turretAngle)));


        // turret axis-bot center offset
        example = example.plus(new Twist2d(new Vector2d(10,0), Math.toRadians(0)));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

                myBot.runAction(myBot.getDrive().actionBuilder(example)
                                        .setTangent(Math.toRadians(90))
                                        .splineToConstantHeading(new Vector2d(-35, -58), Math.toRadians(-90))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
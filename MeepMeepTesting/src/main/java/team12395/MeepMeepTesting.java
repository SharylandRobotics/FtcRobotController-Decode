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
        Pose2d example = new Pose2d(-20, -40, Math.toRadians(25));
        /*

        double cameraOffset = 5;
        double turretAngle = -45;





        // camera-turret axis offset
        example = example.plus(new Twist2d(new Vector2d(-cameraOffset,0), Math.toRadians(0)));


        // turret rotation
        example = example.plus(new Twist2d(new Vector2d(0,0), Math.toRadians(turretAngle)));


        // turret axis-bot center offset
        example = example.plus(new Twist2d(new Vector2d(10,0), Math.toRadians(0)));

         */

        Pose2d target = new Pose2d(-70, 56, Math.toRadians(0));
        double deg = Math.atan2(target.position.y - example.position.y,
                target.position.x - example.position.x) - example.heading.imag;



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

                myBot.runAction(myBot.getDrive().actionBuilder(example)
                                .turn(deg)
                                .splineToConstantHeading(target.position, deg + example.heading.imag)
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
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
        /*
        Pose2d example = new Pose2d(-20, -40, Math.toRadians(25));

        double cameraOffset = 5;
        double turretAngle = -45;

        // camera-turret axis offset
        example = example.plus(new Twist2d(new Vector2d(-cameraOffset,0), Math.toRadians(0)));


        // turret rotation
        example = example.plus(new Twist2d(new Vector2d(0,0), Math.toRadians(turretAngle)));


        // turret axis-bot center offset
        example = example.plus(new Twist2d(new Vector2d(10,0), Math.toRadians(0)));

        Pose2d target = new Pose2d(-70, 56, Math.toRadians(0));
        double deg = Math.atan2(target.position.y - example.position.y,
                target.position.x - example.position.x) - example.heading.imag;
         */
        Pose2d startPose = new Pose2d(-47, 50, Math.toRadians(125));

        Pose2d shoot1 =  new Pose2d(-20, 22, Math.toRadians(90));

        Pose2d preIntake1 = new Pose2d(-11, 28, Math.toRadians(90));
        Pose2d postIntake1 = new Pose2d(preIntake1.position.x, 55, Math.toRadians(90));

        Pose2d openGate = new Pose2d(-1, postIntake1.position.y, Math.toRadians(90));

        Pose2d preIntake2 = new Pose2d(12, 28, Math.toRadians(90));
        Pose2d postIntake2 = new Pose2d(preIntake2.position.x, postIntake1.position.y, Math.toRadians(90));

        Pose2d preIntake3 = new Pose2d(36, 28, Math.toRadians(90));
        Pose2d postIntake3 = new Pose2d(preIntake3.position.x, postIntake1.position.y, Math.toRadians(90));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16.223)
                .build();

                myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                                .setTangent(Math.atan2(shoot1.position.y - startPose.position.y, shoot1.position.x - startPose.position.x))
                                //.lineToXLinearHeading(-28, Math.toRadians(180))
                                .lineToXLinearHeading(shoot1.position.x, shoot1.heading)
                        // shoot
                                .setTangent(0)
                                .lineToX(preIntake1.position.x)
                        //start intake
                                .setTangent(Math.toRadians(90))
                                .lineToY(postIntake1.position.y)
                        // stop intake

                                .setTangent(Math.toRadians(-90))
                                .splineToConstantHeading(openGate.position, Math.toRadians(90))
                            .setTangent(Math.atan2(shoot1.position.y - openGate.position.y, shoot1.position.x - openGate.position.x))
                        // wait for gate

                                //.setTangent(Math.atan2(shoot1.position.y - postIntake1.position.y, shoot1.position.x - postIntake1.position.x))
                                .lineToX(shoot1.position.x)
                        // shoot
                                .setTangent(0)
                                .lineToX(preIntake2.position.x)
                        // start intake
                                .setTangent(Math.toRadians(90))
                                .lineToY(postIntake2.position.y)
                        // stop intake
                                .setTangent(Math.atan2(shoot1.position.y - postIntake2.position.y, shoot1.position.x - postIntake2.position.x))
                                .lineToX(shoot1.position.x)
                        // shoot
                                .setTangent(0)
                                .lineToX(preIntake3.position.x)
                        // start intake
                                .setTangent(Math.toRadians(90))
                                .lineToY(postIntake3.position.y)
                        // stop intake
                                .setTangent(Math.atan2(shoot1.position.y - postIntake3.position.y, shoot1.position.x - postIntake3.position.x))
                                .lineToX(shoot1.position.x)
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
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

                myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-48.5, -49.5, Math.toRadians(-125)))
                        /*
                        .lineTo(new Vector2d(-25, -17.3))
                        //
                        .lineToLinearHeading(new Pose2d(-12, -29, Math.toRadians(-90)))
                        // start pick up
                        .lineTo(new Vector2d(-12, -44.2), new TranslationalVelocityConstraint(10), null) // end pick up
                        .lineTo(new Vector2d(-12, -23))
                        // shoot 2nd volley

                        .lineTo(new Vector2d(12, -29))
                        // start pick up
                        .lineTo(new Vector2d(12, -44.2),  new TranslationalVelocityConstraint(10), null) // end pick up
                        .lineTo(new Vector2d(-12, -23))

                        .lineTo(new Vector2d(34.5, -29))
                        // shoot 3rd volley

                         */
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
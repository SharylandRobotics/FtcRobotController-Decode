package team13581;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(13.625,17 )
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-49, 51, -180))
                //PreLoad
                .setTangent(Math.atan2(51-33.3,-49+32.4))
                .lineToYLinearHeading(32.4,Math.toRadians(135))
                //First Row
                .setTangent(Math.atan2(33.3-28,-32.4+9))
                        .lineToYLinearHeading(28,Math.toRadians(90))
                .setTangent(Math.atan2(55-26.6,0))
                .lineToYLinearHeading(55,Math.toRadians(90))
                //Shoot First Row
                                .setTangent(Math.atan2(51-28,-9+31))
                                .lineToYLinearHeading(32.4, Math.toRadians(135))
                //Second Row
                                .setTangent(Math.atan2(34.7-32.4,11.8+31))
                                .splineToLinearHeading( new Pose2d(11.8, 31.4, Math.toRadians(90)), Math.toRadians(90))
                                .setTangent(Math.atan2(55-31.4,0))
                                .lineToYLinearHeading(55,Math.toRadians(90))
               //Shoot Second Row
                                .setTangent(Math.atan2(33-23.6,29-11.8))
                                .lineToYLinearHeading(32.4, Math.toRadians(135))
                //clear gate
                                .setTangent(Math.atan2(60-33,1+35))
                                .lineToYLinearHeading(55,Math.toRadians(90))
                //Third Row (Needs work on Spline)
                               .setTangent(Math.atan2(27-12,36-17.2))
                .splineToLinearHeading( new Pose2d(36, 27, Math.toRadians(90)), Math.toRadians(90))
                .setTangent(Math.atan2(55-27,0))
                .lineToYLinearHeading(55,Math.toRadians(90))
                //Shoot Third Row
                .setTangent(Math.atan2(33-23.6,40-11.8))
                .lineToYLinearHeading(32.4, Math.toRadians(135))


                                //.lineToYLinearHeading(27,Math.toRadians(90))

                //.lineToX(-12.2)
                //.turn(Math.toRadians(-38))
                //.lineToY(63)
                        //.setTangent(Math.atan2(y2-y1, x2-x1))
                //.LineToYLinearHeading(Y,Math.toRadians())
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
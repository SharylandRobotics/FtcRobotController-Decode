package team13581;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(13.625,17.5 )
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-49, 51, -180))
                //PreLoad
                .setTangent(Math.atan2(27.7-51,-27+49))
                        .lineToYLinearHeading(27.7,Math.toRadians(130))
                //Second row
                .setTangent(Math.atan2(24-27.7,12.2+27))
                .lineToYLinearHeading(24,Math.toRadians(90))
                .setTangent(Math.atan2(56-24,0))
                .lineToYLinearHeading(56,Math.toRadians(90))
                //Second Row Shoot
                        .lineToY(30)
                .setTangent(Math.atan2(27.7-30,-27-12.2))
                .lineToYLinearHeading(27.7, Math.toRadians(130))
                //Clear Gate/Intake
                .splineToConstantHeading(new Vector2d(13,38),Math.toRadians(130))
                .splineToConstantHeading(new Vector2d(12.2,60),Math.toRadians(130))

                //Shoot from Gate
                .splineToConstantHeading(new Vector2d(13,38),Math.toRadians(130))
                .splineToConstantHeading(new Vector2d(-27,27.7),Math.toRadians(130))

                //Clear Gate/Intake
                //.splineToConstantHeading(new Vector2d(13,38),Math.toRadians(130))
                       //.splineToConstantHeading(new Vector2d(12.2,60),Math.toRadians(130))
                //.setTangent((Math.atan2(60-38,12.2-13)))
                //.lineToYLinearHeading(60,Math.toRadians(130))

                //Shoot from Gate
                //.splineToConstantHeading(new Vector2d(13,38),Math.toRadians(130))
                //.setTangent((Math.atan2(38-60,13-12.2)))
                //.lineToYLinearHeading(38,Math.toRadians(130))
                //.splineToConstantHeading(new Vector2d(-27,27.7),Math.toRadians(130))
                //First Row
                .setTangent(Math.atan2(22-27.7,-12+27))
                        .lineToYLinearHeading(22,Math.toRadians(90))
                        .setTangent(Math.atan2(56-28,0))
                        .lineToYLinearHeading(56,Math.toRadians(90))
                //First Row Shoot
                .setTangent(Math.atan2(27.7-56,-27+12))
                .lineToYLinearHeading(27.7,Math.toRadians(130))
                //Tele Position
                .setTangent(Math.atan2(28-27.7,0+27))
                .lineToYLinearHeading(28,Math.toRadians(90))
                //.splineToLinearHeading(new Pose2d(-12, 22, Math.toRadians(90)),Math.toRadians(90))

                               //.lineToY(56)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
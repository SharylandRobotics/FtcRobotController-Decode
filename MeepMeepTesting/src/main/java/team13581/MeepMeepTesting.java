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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-49, -51, 180))
                //PreLoad
                .setTangent(Math.atan2(-33.3+51,-32.4+49))
                .lineToYLinearHeading(-33.3,Math.toRadians(220))
                //First Row
                .setTangent(Math.atan2(-28+33.3,-12+32.4))
                .lineToYLinearHeading(-28,Math.toRadians(270))
                .setTangent(Math.atan2(-56+28,0))
                .lineToYLinearHeading(-56,Math.toRadians(270))
                //First Shoot
                .setTangent(Math.atan2(-33.3+56,-32.4+12))
                .lineToYLinearHeading(-33.3,Math.toRadians(220))
                //Clear Gate
                .setTangent((Math.atan2(-46+33.3,1.4+32.4)))
                .lineToYLinearHeading(-46,Math.toRadians(270))
                .setTangent((Math.atan2(-60+46,0)))
                .lineToYLinearHeading(-60,Math.toRadians(270))
                //Second Row
                .setTangent(Math.atan2(-28+55,0))
                .lineToYLinearHeading(-28,Math.toRadians(270))
                .setTangent(Math.atan2(0,12.2-1.4))
                .lineToXLinearHeading(12.2,Math.toRadians(270))
                .setTangent(Math.atan2(-55+28,0))
                .lineToYLinearHeading(-55,Math.toRadians(270))
                //Second Shoot
                .setTangent(Math.atan2(-33.3+55,-32.4-12.2))
                .lineToYLinearHeading(-33.3, Math.toRadians(220))
                //Third Row
                .setTangent(Math.atan2(-28+33.3,36+32.4))
                .lineToYLinearHeading(-28,Math.toRadians(270))
                .setTangent(Math.atan2(-55+28,0))
                .lineToYLinearHeading(-55,Math.toRadians(270))
                //Third Shoot
                .setTangent(Math.atan2(-33.3+55,-32.4-36))
                .lineToYLinearHeading(-33.3,Math.toRadians(220))
                //TeleOp Position
                .setTangent(Math.atan2(-28+33.3,-8+32.4))
                .lineToYLinearHeading(-28,Math.toRadians(270))






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
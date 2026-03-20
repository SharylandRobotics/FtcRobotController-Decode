package team13581;

import com.acmerobotics.roadrunner.Action;
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
                .setDimensions(13.625,18 )
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-49, -51, Math.toRadians(180)))
                //PreLoad
                .setTangent(Math.atan2(-33.3+51,-32.4+49))
                .lineToYLinearHeading(-33.3,Math.toRadians(-130))
                //First Row
                .setTangent(Math.atan2(-20+33.3,-12+32.4))
                .lineToYLinearHeading(-20,Math.toRadians(-90))
                .setTangent(Math.atan2(-56+28,0))
                .lineToYLinearHeading(-56,Math.toRadians(-90))
                //FirstShoot
                .setTangent(Math.atan2(-33.3+56,-32.4+12))
                .lineToYLinearHeading(-33.3,Math.toRadians(-130))
                //Clear Gate
                .setTangent((Math.atan2(-46+33.3,1.4+32.4)))
                .lineToYLinearHeading(-46,Math.toRadians(-90))
                .setTangent((Math.atan2(-60+46,0)))
                .lineToYLinearHeading(-60,Math.toRadians(-90))
                //Second Row
                .setTangent(Math.atan2(-28+55,0))
                .lineToYLinearHeading(-28,Math.toRadians(-90))
                .setTangent(Math.atan2(0,12.2-1.4))
                .lineToXLinearHeading(12.2,Math.toRadians(-90))
                .setTangent(Math.atan2(-55+28,0))
                .lineToYLinearHeading(-55,Math.toRadians(-90))
                .setTangent(Math.atan2(-40+55,0))
                .lineToYLinearHeading(-40,Math.toRadians(-90))
                //Second Shoot
                .setTangent(Math.atan2(-33.3+40,-32.4-12.2))
                .lineToYLinearHeading(-33.3, Math.toRadians(-130))
                //Third Row
                .setTangent(Math.atan2(-28+33.3,36+32.4))
                .lineToYLinearHeading(-28,Math.toRadians(-90))
                .setTangent(Math.atan2(-55+28,0))
                .lineToYLinearHeading(-55,Math.toRadians(-90))
                //Third Shoot
                .setTangent(Math.atan2(-33.3+55,-32.4-36))
                .lineToYLinearHeading(-33.3,Math.toRadians(-130))
                //Tele
                .setTangent(Math.atan2(-28+33.3,-8+32.4))
                .lineToYLinearHeading(-28,Math.toRadians(-90))











                //Preload
                //.setTangent(Math.atan2(19.5-7,55-62))
                //.lineToYLinearHeading(19.5,Math.toRadians(157))
                //Third Row
                //.setTangent(Math.atan2(28.5-19.5,35.5-55))
                //.lineToYLinearHeading(28.5,Math.toRadians(90))
                //.setTangent(Math.atan2(58,0))
                //.lineToYLinearHeading(58,Math.toRadians(90))
                //Third Shoot
                //.setTangent(Math.atan2(19.5-58,55-35.5))
                //.lineToYLinearHeading(19.5,Math.toRadians(157))
                //Human Zone
                //.setTangent(Math.atan2(60-19.5,59-55))
                //.lineToYLinearHeading(60,Math.toRadians(75))
                //Human Shoot
                //.setTangent(Math.atan2(19.5-60,55-59))
                //.lineToYLinearHeading(19.5,Math.toRadians(157))
                //Blue Tape
                //.setTangent((Math.atan2(60-19.5,47-55)))
                //.lineToYLinearHeading(60,Math.toRadians(90))
                //Tape Shoot
                //.setTangent(Math.atan2(19.5-60,55-47))
                //.lineToYLinearHeading(19.5,Math.toRadians(157))
                //Tele
                //.setTangent(Math.atan2(22-19.5,46-55))
                //.lineToYLinearHeading(22,Math.toRadians(90))
                    .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
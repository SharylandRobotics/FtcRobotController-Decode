package team13580;

import com.acmerobotics.roadrunner.*;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.lang.Math;

public class NONMotid_blue {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(17.7, 17.8)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-49, -51, Math.toRadians(-126)))
                //path1
                .setTangent(Math.atan2(-24+51, -30+49))
                .lineToY(-24)
                //path2
                .setTangent(Math.atan2(-28+24, 15+30))
                .lineToYLinearHeading(-28, Math.toRadians(-90))
                .setTangent(Math.atan2(-34+28, 0))
                .lineToYLinearHeading(-34, Math.toRadians(-90))
                .setTangent(Math.atan2(0, 11-15))
                .lineToX(11)
                .setTangent(Math.atan2(-54+34, 0))
                .lineToYLinearHeading(-54, Math.toRadians(-90))
                //path3
                .setTangent(Math.atan2(-28+54, 0))
                .lineToYLinearHeading(-28, Math.toRadians(-90))
                .setTangent(Math.atan2(-24+28, -30-11))
                .lineToYLinearHeading(-24, Math.toRadians(-175))
                .turnTo(Math.toRadians(-130))
                .waitSeconds(2)
                //path4
                .setTangent(Math.atan2(-28+24, -10+30))
                .lineToYLinearHeading(-28, Math.toRadians(-90))
                .setTangent(Math.atan2(-38+28, 0))
                .lineToYLinearHeading(-38, Math.toRadians(-90))
                .setTangent(Math.atan2(0, -10+10))
                .lineToX(-14)
                .setTangent(Math.atan2(-53+38, 0))
                .lineToYLinearHeading(-53, Math.toRadians(-90))

                //path5
                .setTangent(Math.atan2(-24+53, -30+14))
                .lineToYLinearHeading(-24, Math.toRadians(-130))
                .waitSeconds(2)
                //path6
                .setTangent(Math.atan2(-28+24, 38+30))
                .lineToYLinearHeading(-28, Math.toRadians(-90))
                .setTangent(Math.atan2(-34+28, 0))
                .lineToYLinearHeading(-34, Math.toRadians(-90))
                .setTangent(Math.atan2(0, 34-38))
                .lineToX(34)
                .setTangent(Math.atan2(-56+34, 0))
                .lineToYLinearHeading(-56, Math.toRadians(-90))
                //path7
                .setTangent(Math.atan2(-24+56, -30-34))
                .lineToYLinearHeading(-24, Math.toRadians(-130))
                .waitSeconds(2)
                //path8
                .setTangent(Math.atan2(-40+24, -10+30))
                .lineToYLinearHeading(-40, Math.toRadians(-90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}

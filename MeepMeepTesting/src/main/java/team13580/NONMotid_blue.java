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
                .lineToYLinearHeading(-24, Math.toRadians(-126))
                //path2
                .setTangent(Math.atan2(-20+24, 12+30))
                .lineToYLinearHeading(-20, Math.toRadians(-90))
                .setTangent(Math.atan2(-53+20, 0))
                .lineToYLinearHeading(-53, Math.toRadians(-90), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-60, 100))
                .setTangent(Math.atan2(-16+53, 0))
                .lineToYLinearHeading(-16, Math.toRadians(-90), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-60, 100))

                //path3
                .setTangent(Math.atan2(-18+16, -20-12))
                .lineToYLinearHeading(-18, Math.toRadians(-135), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-60, 100))

                .waitSeconds(2)
                //path4
                .setTangent(Math.atan2(-20+18, 18+20))
                .lineToYLinearHeading(-20, Math.toRadians(-120))
                .setTangent(Math.atan2(-58+20, 0))
                .lineToYLinearHeading(-58, Math.toRadians(-115), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                        //slide to intake
                . setTangent(Math.atan2(0, 19-18))
                .lineToX(19, new TranslationalVelConstraint(60), new ProfileAccelConstraint(-60, 100))

                //path5
                .setTangent(Math.atan2(-16+58, 0))
                .lineToYLinearHeading(-16, Math.toRadians(-115), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-60, 100))
                .setTangent(Math.atan2(-24+16, -22-19))
                .lineToYLinearHeading(-24, Math.toRadians(-140), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                .waitSeconds(2)
                //path6
                .setTangent(Math.atan2(-18+24, -14+22))
                .lineToYLinearHeading(-18, Math.toRadians(-90))
                .setTangent(Math.atan2(-48+18, 0))
                .lineToYLinearHeading(-48, Math.toRadians(-90), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-60, 100))
                 //path 7
                .setTangent(Math.atan2(-18+48, -16+14))
                .lineToYLinearHeading(-18, Math.toRadians(-140), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                                .waitSeconds(2)
                //path 8
                .setTangent(Math.atan2(-49+18, -14+16))
                .lineToYLinearHeading(-49, Math.toRadians(-90), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-60, 100))



                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}

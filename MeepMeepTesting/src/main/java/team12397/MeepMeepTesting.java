package team12397;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import javax.imageio.ImageIO;
import java.awt.*;
import java.io.File;
import java.io.IOException;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(60, -12, Math.toRadians(-90)))
                        // shoot & scan
                        .lineTo(new Vector2d(35, -28))
                        // start pick up
                        .lineTo(new Vector2d(35, -43)) // end pick up
                        .lineTo(new Vector2d(60, -12))
                        // shoot 2nd volley
                        .lineTo(new Vector2d(12, -28))
                        // start pick up
                        .lineTo(new Vector2d(12, -43)) // end pick up

                        .build());


        Image img = null;
        try { img = ImageIO.read(new File("C:\\Users\\shsrobotics\\Downloads\\field-2025-official.png")); }
        catch(IOException e) {}

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
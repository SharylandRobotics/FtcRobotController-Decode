package org.firstinspires.ftc.team13581.Autonomous;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team13581.RobotHardware;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import org.firstinspires.ftc.team13581.rr.MecanumDrive;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;

import java.lang.Math;

@Config
@Autonomous(name="RR Red Short Goal", group="Autonomous")

public class RRRedShortGoal extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    class NearShoot implements Action {
        private boolean initialized = false;
        private double parameter = 0;
        public NearShoot(double val){
            parameter = val;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            robot.setStopper(1);
            robot.setAimPos(0.6);
            robot.setShootSpeed(1300);
            return false;
        }
    }
    class Intake implements Action {
        private boolean initialized = false;
        private double parameter = 0;

        public Intake(double val){
            parameter = val;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            robot.setStopper(0);
            robot.setIntake1(1);
            robot.setIntake2(.7);
            return false;
        }
    }

    public Action nearShoot(double value){
        return new NearShoot(value);
    }
    public Action Intake(double value){
        return Intake(value);
    }
    @Override
    public void runOpMode(){

        Pose2d initialPose = new Pose2d(-49, 51, -180);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        robot.init();

        Action Preload = drive.actionBuilder(initialPose)
                .setTangent(Math.atan2(51-33.3,-49+32.4))
                .lineToYLinearHeading(32.4,Math.toRadians(135))
                .build();

        Action FirstRow = drive.actionBuilder(initialPose)
                .setTangent(Math.atan2(33.3-28, -32.4+9))
                .lineToYLinearHeading(28, Math.toRadians(90))
                .setTangent(Math.atan2(55-26.6,0))
                .lineToYLinearHeading(55,Math.toRadians(90))
                . build();
        Action FirstShoot = drive.actionBuilder(initialPose)
                .setTangent(Math.atan2(51-28,-9+31))
                .lineToYLinearHeading(32.4,Math.toRadians(135))
                .build();
        Action SecondRow = drive.actionBuilder(initialPose)
                .setTangent(Math.atan2(34.7-32.4,11.8+31))
                .splineToLinearHeading( new Pose2d(11.8, 31.4, Math.toRadians(90)), Math.toRadians(90))
                .setTangent(Math.atan2(55-31.4,0))
                .lineToYLinearHeading(55,Math.toRadians(90))
                .build();
        Action SecondShoot = drive.actionBuilder(initialPose)
                .setTangent(Math.atan2(33-23.6,29-11.8))
                .lineToYLinearHeading(32.4, Math.toRadians(135))
                .build();
        Action ClearGate = drive.actionBuilder(initialPose)
                .setTangent(Math.atan2(60-33,1+35))
                .lineToYLinearHeading(55,Math.toRadians(90))
                .build();
        Action ThirdRow = drive.actionBuilder(initialPose)

                .build();
        while(opModeInInit()) {
            // Display heading and status continuously during init loop
            telemetry.addData("Status", "Hardware Initialized");
            telemetry.addData("Heading", "%4.0f", robot.getHeading());
            telemetry.update();
        }

        // Wait for PLAY; exit early if stop is pressed
        waitForStart();
        if (isStopRequested()) return;

        // Execute full autonomous path sequence once started
        Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    nearShoot(1),
                                    Preload
                            ),
                            Intake(1),
                            new SleepAction(.1)

                    )
            );

            // Indicate completion and pause for display
            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);  // Pause to display last telemetry message.

    }
    private Action sleepAction(long milliseconds) {
        return packet -> {
            sleep(milliseconds);
            return false;
        };
    }
}
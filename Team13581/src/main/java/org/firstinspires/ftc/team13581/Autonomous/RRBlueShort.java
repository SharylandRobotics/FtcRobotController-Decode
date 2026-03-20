package org.firstinspires.ftc.team13581.Autonomous;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team13581.RobotHardware;
import org.firstinspires.ftc.team13581.rr.MecanumDrive;

import java.lang.Math;

@Config
@Autonomous(name="RR Blue Short Comp", group="Autonomous")

public class RRBlueShort extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    class NearShoot implements Action {
        private boolean initialized = false;
        private double parameter = 0;
        public NearShoot(double val){
            parameter = val;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            robot.setHoodPos(0.6);
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
            robot.setIntake1(1);
            robot.setIntake2(.7);
            return false;
        }
    }
    class StopperUp implements Action {
        private boolean initialized = false;
        private double parameter = 0;

        public StopperUp(double val){
            parameter = val;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            robot.setStopper(.9);
            return false;
        }
    }
    class StopShoot implements Action {
        private boolean initialized = false;
        private double parameter = 0;
        public StopShoot(double val){
            parameter = val;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            robot.setHoodPos(0.6);
            robot.setShootSpeed(0);
            return false;
        }
    }
    class StopperDown implements Action {
        private boolean initialized = false;
        private double parameter = 0;
        public StopperDown(double val){
            parameter = val;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            robot.setStopper(0.4);
            return false;
        }
    }
    class StopIntake implements Action {
        private boolean initialized = false;
        private double parameter = 0;
        public StopIntake(double val){
            parameter = val;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            robot.setIntake1(0);
            robot.setIntake2(0);
            return false;
        }
    }
    class FirstShot implements Action {
        private boolean initialized = false;
        private double parameter = 0;
        public FirstShot(double val){
            parameter = val;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            robot.setHoodPos(0.6);
            robot.setShootSpeed(1250);
            return false;
        }
    }
    public Action StopperUp(double value){
        return new StopperUp(value);
    }
    public Action nearShoot(double value){
        return new NearShoot(value);
    }
    public Action Intake(double value){
        return new Intake(value);
    }
    public Action StopperDown(double value){
        return new StopperDown(value);
    }
    public Action StopIntake(double value){
        return new StopIntake(value);
    }
    public Action StopShoot(double value){
        return new StopShoot(value);
    }
    public Action FirstShot(double value){return new FirstShot(value);}
    @Override
    public void runOpMode(){

        Pose2d initialPose = new Pose2d(-49, -51, 180);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        robot.init();

        Action Preload = drive.actionBuilder(new Pose2d(-49, -51, 180))
                .setTangent(Math.atan2(-33.3+51,-32.4+49))
                .lineToYLinearHeading(-33.3,Math.toRadians(-130))
                .build();

        Action FirstRow = drive.actionBuilder(new Pose2d(-32.4,-33.3, Math.toRadians(-130)))
                .setTangent(Math.atan2(-20+33.3,-12+32.4))
                .lineToYLinearHeading(-20,Math.toRadians(-90))
                .setTangent(Math.atan2(-56+28,0))
                .lineToYLinearHeading(-56,Math.toRadians(-90))
                . build();
        Action FirstShoot = drive.actionBuilder(new Pose2d(-12,-56,Math.toRadians(-90)))
                .setTangent(Math.atan2(-33.3+56,-32.4+12))
                .lineToYLinearHeading(-33.3,Math.toRadians(-130))
                .build();
        Action SecondRow = drive.actionBuilder(new Pose2d(1.4,-60, Math.toRadians(-90)))
                .setTangent(Math.atan2(-24+55,6+1.4))
                .lineToYLinearHeading(-24,Math.toRadians(-90))
                .setTangent(Math.atan2(0,12.2-6))
                .lineToXLinearHeading(12.2,Math.toRadians(-90))
                .setTangent(Math.atan2(-55+28,0))
                .lineToYLinearHeading(-55,Math.toRadians(-90))
                .setTangent(Math.atan2(-28+55,0))
                .lineToYLinearHeading(-28,Math.toRadians(-90))
                .build();
        Action SecondShoot = drive.actionBuilder(new Pose2d(12.2,-28, Math.toRadians(-90)))
                .setTangent(Math.atan2(-33.3+28,-32.4-12.2))
                .lineToYLinearHeading(-33.3, Math.toRadians(-130))
                .build();
        Action ClearGate = drive.actionBuilder(new Pose2d(-32.4,-33.3, Math.toRadians(-130)))
                .setTangent((Math.atan2(-46+33.3,1.4+32.4)))
                .lineToYLinearHeading(-46,Math.toRadians(-90))
                .setTangent((Math.atan2(-60+46,0)))
                .lineToYLinearHeading(-60,Math.toRadians(-90))
                .build();
        Action ThirdRow = drive.actionBuilder(new Pose2d(-32.4,-33.3, Math.toRadians(-130)))
                .setTangent(Math.atan2(-28+33.3,36+32.4))
                .lineToYLinearHeading(-28,Math.toRadians(-90))
                .setTangent(Math.atan2(-55+28,0))
                .lineToYLinearHeading(-55,Math.toRadians(-90))
                .build();
        Action ThirdShoot = drive.actionBuilder(new Pose2d(36,-55, Math.toRadians(-90)))
                .setTangent(Math.atan2(-33.3+55,-32.4-36))
                .lineToYLinearHeading(-33.3,Math.toRadians(-130))
                .build();
        Action TelePosition = drive.actionBuilder(new Pose2d(-32.4,-33.3, Math.toRadians(-130)))
                .setTangent(Math.atan2(-28+33.3,-8+32.4))
                .lineToYLinearHeading(-28,Math.toRadians(-90))
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
                                    StopperUp(1),
                                    FirstShot(1),
                                    Preload
                            ),
                            new SleepAction(.4),
                            Intake(1),
                            new SleepAction(1),
                            StopIntake(1),

                            new ParallelAction(
                                    StopperDown(1),
                                    nearShoot(1),
                                    Intake(1),
                                    FirstRow
                            ),
                            StopIntake(1),
                            new ParallelAction(
                                    StopperUp(1),
                                    FirstShoot
                            ),
                            Intake(1),
                            new SleepAction(.4),
                            new ParallelAction(
                                    StopperDown(1),
                                    StopIntake(1),
                                    ClearGate
                            ),
                            new ParallelAction(
                                    Intake(1),
                                    SecondRow
                            ),
                            StopIntake(1),
                            new ParallelAction(
                                    StopperUp(1),
                                    SecondShoot
                            ),
                            Intake(1),
                            new SleepAction(.4),
                            new ParallelAction(
                                    StopperDown(1),
                                    ThirdRow
                            ),
                            StopIntake(1),
                            new ParallelAction(
                                    StopperUp(1),
                                    ThirdShoot
                            ),
                            Intake(1),
                            new SleepAction(.5),

                            TelePosition



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
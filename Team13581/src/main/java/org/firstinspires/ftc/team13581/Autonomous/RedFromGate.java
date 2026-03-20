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
@Autonomous(name="Red Gate", group="Autonomous")

public class RedFromGate extends LinearOpMode {

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
            robot.setIntake2(1);
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
            robot.setStopper(1);
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

        Pose2d initialPose = new Pose2d(-49, 51, -180);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        robot.init();

        Action Preload = drive.actionBuilder(initialPose)
                .setTangent(Math.atan2(27.7-51,-27+49))
                .lineToYLinearHeading(27.7,Math.toRadians(130))
                .build();

        Action SecondRow = drive.actionBuilder(new Pose2d(-27,27.7, Math.toRadians(130)))
                .setTangent(Math.atan2(24-27.7,12.2+27))
                .lineToYLinearHeading(24,Math.toRadians(90))
                .setTangent(Math.atan2(56-24,0))
                .lineToYLinearHeading(56,Math.toRadians(90))
                . build();
        Action SecondRowShoot = drive.actionBuilder(new Pose2d(12.2,56,Math.toRadians(90)))
                .lineToY(30)
                .setTangent(Math.atan2(27.7-30,-27-12.2))
                .lineToYLinearHeading(27.7, Math.toRadians(130))
                .build();
        Action ClearGate = drive.actionBuilder(new Pose2d(-27,27.7, Math.toRadians(130)))
                .splineToConstantHeading(new Vector2d(13,38),Math.toRadians(130))
                .splineToConstantHeading(new Vector2d(12.2,63),Math.toRadians(130))
                .build();
        Action GateShoot = drive.actionBuilder(new Pose2d(12.2,63, Math.toRadians(130)))
                .splineToConstantHeading(new Vector2d(13,38),Math.toRadians(130))
                .splineToConstantHeading(new Vector2d(-27,27.7),Math.toRadians(130))
                .build();
        Action FirstRow = drive.actionBuilder(new Pose2d(-27,27.7, Math.toRadians(130)))
                .setTangent(Math.atan2(22-27.7,-12+27))
                .lineToYLinearHeading(22,Math.toRadians(90))
                .setTangent(Math.atan2(56-28,0))
                .lineToYLinearHeading(56,Math.toRadians(90))
                .build();
        Action FirstShoot = drive.actionBuilder(new Pose2d(-12,56, Math.toRadians(90)))
                .setTangent(Math.atan2(27.7-56,-27+12))
                .lineToYLinearHeading(27.7,Math.toRadians(130))
                .build();
        Action TelePosition = drive.actionBuilder(new Pose2d(-27,27.7, Math.toRadians(130)))
                .setTangent(Math.atan2(28-27.7,0+27))
                .lineToYLinearHeading(28,Math.toRadians(90))
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
                                    Intake(1),
                                    nearShoot(1),
                                    SecondRow
                            ),
                            StopIntake(1),
                            new ParallelAction(
                                    StopperUp(1),
                                    SecondRowShoot
                            ),
                            Intake(1),
                            new SleepAction(.4),
                            new ParallelAction(
                                    StopperDown(1),
                                    Intake(1),
                                    ClearGate
                            ),
                            new SleepAction(.5),
                            new ParallelAction(
                                    StopIntake(1),
                                    StopperUp(1),
                                    GateShoot
                            ),
                            Intake(1),
                            new SleepAction(.4),
                            new ParallelAction(
                                    StopperDown(1),
                                    ClearGate
                            ),
                            new SleepAction(.5),

                            new ParallelAction(
                                    StopIntake(1),
                                    StopperUp(1),
                                    GateShoot
                            ),
                            Intake(1),
                            new SleepAction(.4),
                            new ParallelAction(
                                    StopperDown(1),
                                    FirstRow
                            ),
                            new ParallelAction(
                                    StopIntake(1),
                                    StopperUp(1),
                                    FirstShoot
                            ),
                            Intake(1),
                            new SleepAction(.4),
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
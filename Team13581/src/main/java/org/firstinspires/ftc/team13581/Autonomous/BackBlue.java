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
@Autonomous(name="Blue Backside", group="Autonomous")

public class BackBlue extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    class Shoot implements Action {
        private boolean initialized = false;
        private double parameter = 0;
        public Shoot(double val){
            parameter = val;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            robot.setHoodPos(0.45);
            robot.setShootSpeed(1925);
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
            robot.setHoodPos(0.55);
            robot.setShootSpeed(1925);
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
            robot.setStopper(.7);
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
            robot.setHoodPos(0.45);
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
            robot.setStopper(0.2);
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
    class None implements Action {
        private boolean initialized = false;
        private double parameter = 0;
        public None(double val){
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
    public Action Shoot(double value){
        return new Shoot(value);
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

        Pose2d initialPose = new Pose2d(62, -7, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        robot.init();

        Action Preload = drive.actionBuilder(initialPose)
                .setTangent(Math.atan2(-19.5+7,55-62))
                .lineToYLinearHeading(-19.5,Math.toRadians(-163))
                .build();
        Action ThirdRow = drive.actionBuilder(new Pose2d(55,-19.5, Math.toRadians(-163)))
                .setTangent(Math.atan2(-28.5+19.5,35.5-55))
                .lineToYLinearHeading(-28.5,Math.toRadians(-90))
                .setTangent(Math.atan2(-58,0))
                .lineToYLinearHeading(-58,Math.toRadians(-90))
                .build();
        Action ThirdShoot = drive.actionBuilder(new Pose2d(35,-58, Math.toRadians(-90)))
                .setTangent(Math.atan2(-19.5+58,55-35.5))
                .lineToYLinearHeading(-19.5,Math.toRadians(-163))
                .build();
        Action HumanZone = drive.actionBuilder(new Pose2d(55,-19.5, Math.toRadians(-163)))
                .setTangent(Math.atan2(-60+19.5,59-55))
                .lineToYLinearHeading(-60,Math.toRadians(-75))
                . build();
        Action HumanShoot = drive.actionBuilder(new Pose2d(59,-60,Math.toRadians(-75)))
                .setTangent(Math.atan2(-19.5+60,55-59))
                .lineToYLinearHeading(-19.5,Math.toRadians(-163))
                .build();
        Action BLueTape = drive.actionBuilder(new Pose2d(55,-19.5, Math.toRadians(-163)))
                .setTangent((Math.atan2(-60+19.5,47-55)))
                .lineToYLinearHeading(-60,Math.toRadians(-90))
                .build();
        Action TapeShoot = drive.actionBuilder(new Pose2d(47,-60, Math.toRadians(-90)))
                .setTangent(Math.atan2(-19.5+60,55-47))
                .lineToYLinearHeading(-19.5,Math.toRadians(-163))
                .build();
        Action TelePosition = drive.actionBuilder(new Pose2d(55,-19.5, Math.toRadians(-163)))
                .setTangent(Math.atan2(-22+19.5,46-55))
                .lineToYLinearHeading(-22,Math.toRadians(-90))
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
                            new SleepAction(2),
                            Intake(1),
                            new SleepAction(.4),
                            StopperDown(1),

                            new ParallelAction(
                                    Shoot(1),
                                    ThirdRow,
                                    Intake(1)
                            ),
                            StopIntake(1),
                            new ParallelAction(
                                    ThirdShoot,
                                    StopperUp(1)
                            ),
                            Intake(1),
                            new SleepAction(.4)
                            /*new ParallelAction(
                                    StopperDown(1),
                                    HumanZone
                            ),
                            StopIntake(1),
                            new ParallelAction(
                                    StopperUp(1),
                                    HumanShoot
                            ),
                            Intake(1),
                            new SleepAction(.4)
                            //new ParallelAction(
                                    //StopperDown(1)

                             */














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
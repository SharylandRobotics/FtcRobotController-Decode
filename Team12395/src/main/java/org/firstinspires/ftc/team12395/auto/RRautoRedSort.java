package org.firstinspires.ftc.team12395.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team12395.rr.MecanumDrive;
import org.firstinspires.ftc.team12395.RobotHardware;

import java.lang.Math;

@Autonomous(name="RR Red SORTING", group="Alliance")
public class RRautoRedSort extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    RobotHardware.RoadRunnerActions actionLib = robot.new RoadRunnerActions();
    Pose2d initialPose = new Pose2d(-48.5, 49.5, Math.toRadians(126));
    MecanumDrive drive;
    Pose2d latestPose = initialPose;

    private Action updatePose(){
        return (TelemetryPacket packet) -> {
            drive.updatePoseEstimate();
            latestPose = drive.localizer.getPose();
            return false;
        };
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init();

        drive = robot.standardDrive;

        Pose2d startPose = new Pose2d(-47, 50, Math.toRadians(126));

        Pose2d shoot1 =  new Pose2d(-22, 16, Math.toRadians(90));

        Pose2d preIntake1 = new Pose2d(-8, 22, Math.toRadians(90));
        Pose2d postIntake1 = new Pose2d(preIntake1.position.x, 43, Math.toRadians(90));

        Pose2d openGate = new Pose2d(5, postIntake1.position.y, Math.toRadians(90));

        Pose2d preIntake2 = new Pose2d(17, preIntake1.position.y, Math.toRadians(90));
        Pose2d postIntake2 = new Pose2d(preIntake2.position.x, postIntake1.position.y+4, Math.toRadians(90));

        Pose2d preIntake3 = new Pose2d(39, preIntake1.position.y, Math.toRadians(90));
        Pose2d postIntake3 = new Pose2d(preIntake3.position.x, postIntake1.position.y+8, Math.toRadians(90));

        // return to volley pose

        robot.setLocalizerPosition(startPose);

        Action driveToPLShoot = drive.actionBuilder(startPose)
                .setTangent(Math.atan2(shoot1.position.y - startPose.position.y, shoot1.position.x - startPose.position.x))
                //.lineToXLinearHeading(-28, Math.toRadians(180))
                .lineToXLinearHeading(shoot1.position.x, shoot1.heading)
                .turnTo(shoot1.heading)
                // scan & sort
                // shoot
                .build();

        // init done

        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        robot.setMagManualBulk("GPP");

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                actionLib.setHoodAng(0.9),
                                actionLib.setShooterVel(1400),
                                new SequentialAction(
                                        new RaceAction(
                                                actionLib.setTurretPos(-38),
                                                new SleepAction(2)
                                        ),
                                        actionLib.stopTurretPower()
                                ),

                                driveToPLShoot
                        ),
                        new SleepAction(0.5),
                        actionLib.shootAllBalls(),
                        new SleepAction(1.5),
                        updatePose()
                )
        );

        Action driveToRow1 = drive.actionBuilder(latestPose)
                .setTangent(0)
                .lineToXConstantHeading(preIntake1.position.x)
                //start intake
                .setTangent(Math.toRadians(90))
                .lineToYConstantHeading(postIntake1.position.y)
                .turnTo(postIntake1.heading)
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction( // drive to first row & switch spindexer\
                                driveToRow1,
                                actionLib.setIntakeVel(2600),
                                // scan for motif --
                                new SequentialAction(
                                        new RaceAction(
                                                actionLib.setTurretPos(-110),
                                                new RaceAction(
                                                        new SleepAction(2),
                                                        actionLib.scanMotif()
                                                )
                                        ),
                                        new RaceAction(
                                                actionLib.setTurretPos(-38),
                                                new SleepAction(2)
                                        ),
                                        actionLib.stopTurretPower()
                                )
                                // scan for motif --

                        ),
                        new SleepAction(0.2),



                        //actionLib.sortCurrentSpindexer(),
                        //new SleepAction(2),
                        updatePose()
                )
        );

        Action driveToGate = drive.actionBuilder(latestPose)
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(openGate.position, Math.toRadians(90))
                // wait for gate
                .waitSeconds(0.5)
                .setTangent(Math.atan2(shoot1.position.y - openGate.position.y, shoot1.position.x - openGate.position.x))
                .lineToX(shoot1.position.x)
                .build();

        // open gate
        Actions.runBlocking(
                new SequentialAction(
                        //actionLib.setIntakeVel(0),
                        new ParallelAction(
                                driveToGate,

                                // sorting --
                                new SequentialAction(
                                        // detect balls in 3 sec
                                        new RaceAction(
                                                new ParallelAction(
                                                        actionLib.scanColorToggle(),
                                                        new SequentialAction(
                                                                new SleepAction(0.5),
                                                                actionLib.spindexerTargetAddVel(20, 1000),
                                                                new SleepAction(0.2)
                                                        )
                                                ),
                                                new SleepAction(1.5)
                                        ),

                                        // then sort
                                        actionLib.sortCurrentSpindexer(),
                                        new SleepAction(1),
                                        actionLib.spindexerTargetAddVel(-30,1000),
                                        new SleepAction(0.7)
                                )
                                // sorting --

                        ),

                        actionLib.setIntakeVel(0),
                        //new SleepAction(1),
                        actionLib.shootAllBallsSlow(),
                        new SleepAction(1.5),
                        updatePose()
                )
        );

        Action driveToRow2 = drive.actionBuilder(latestPose)
                .setTangent(0)
                .lineToXConstantHeading(preIntake2.position.x)
                // start intake
                .setTangent(Math.toRadians(90))
                .lineToYConstantHeading(postIntake2.position.y)
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        actionLib.setIntakeVel(2600),
                        driveToRow2,
                        new SleepAction(0.2),
                        updatePose()
                )
        );

        Action driveToShoot2 = drive.actionBuilder(latestPose)
                // stop intake
                .setTangent(Math.atan2(shoot1.position.y - postIntake2.position.y, shoot1.position.x - postIntake2.position.x))
                .lineToX(shoot1.position.x)
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                driveToShoot2,

                                // sorting --
                                new SequentialAction(
                                        // detect balls in 3 sec
                                        new RaceAction(
                                                new ParallelAction(
                                                        actionLib.scanColorToggle(),
                                                        new SequentialAction(
                                                                new SleepAction(0.5),
                                                                actionLib.spindexerTargetAddVel(20, 1000),
                                                                new SleepAction(0.2)
                                                        )
                                                ),
                                                new SleepAction(1.5)
                                        ),

                                        // then sort
                                        actionLib.sortCurrentSpindexer(),
                                        new SleepAction(1),
                                        actionLib.spindexerTargetAddVel(-30, 1000),
                                        new SleepAction(0.7)
                                )
                                // sorting --
                        ),

                        actionLib.setIntakeVel(0),
                        actionLib.shootAllBallsSlow(),
                        new SleepAction(1.5),
                        updatePose()
                )
        );

        Action driveToRow3 = drive.actionBuilder(latestPose)
                .setTangent(0)
                .lineToXConstantHeading(preIntake3.position.x)
                // start intake
                .setTangent(Math.toRadians(90))
                .lineToYConstantHeading(postIntake3.position.y)
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        actionLib.setIntakeVel(2600),
                        driveToRow3,
                        new SleepAction(0.2),
                        updatePose()
                )
        );

        Action driveToShoot3 = drive.actionBuilder(latestPose)
                .setTangent(Math.atan2(shoot1.position.y - postIntake3.position.y, shoot1.position.x - postIntake3.position.x))
                .lineToX(shoot1.position.x)
                .build();

        Action driveOff = drive.actionBuilder(shoot1)
                .setTangent(0)
                .lineToX(preIntake2.position.x)
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                driveToShoot3,
                                // sorting --
                                new SequentialAction(
                                        // detect balls in 3 sec
                                        new RaceAction(
                                                new ParallelAction(
                                                        actionLib.scanColorToggle(),
                                                        new SequentialAction(
                                                                new SleepAction(0.5),
                                                                actionLib.spindexerTargetAddVel(20, 1000),
                                                                new SleepAction(0.2)
                                                        )
                                                ),
                                                new SleepAction(1.5)
                                        ),

                                        // then sort
                                        actionLib.sortCurrentSpindexer(),
                                        new SleepAction(1),
                                        actionLib.spindexerTargetAddVel(-30, 1000),
                                        new SleepAction(0.7)
                                )
                                // sorting --
                        ),

                        actionLib.setIntakeVel(0),
                        actionLib.shootAllBallsSlow(),
                        new SleepAction(1.5),
                        new ParallelAction(
                                driveOff,
                                new RaceAction(
                                        actionLib.setTurretPos(0),
                                        new SleepAction(2)
                                )
                        )
                )
        );
    }
}
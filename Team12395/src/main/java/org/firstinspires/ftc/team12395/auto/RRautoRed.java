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

@Autonomous(name="RR Long Auto Red", group="Alliance")
public class RRautoRed extends LinearOpMode {

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

        Pose2d startPose = new Pose2d(-47, 50, Math.toRadians(125));

        Pose2d shoot1 =  new Pose2d(-20, 20, Math.toRadians(90));

        Pose2d preIntake1 = new Pose2d(-9, 26, Math.toRadians(90));
        Pose2d postIntake1 = new Pose2d(preIntake1.position.x, 51, Math.toRadians(90));

        Pose2d openGate = new Pose2d(1, postIntake1.position.y, Math.toRadians(90));

        Pose2d preIntake2 = new Pose2d(14, preIntake1.position.y, Math.toRadians(90));
        Pose2d postIntake2 = new Pose2d(preIntake2.position.x, postIntake1.position.y, Math.toRadians(90));

        Pose2d preIntake3 = new Pose2d(38, preIntake1.position.y, Math.toRadians(90));
        Pose2d postIntake3 = new Pose2d(preIntake3.position.x, postIntake1.position.y, Math.toRadians(90));

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
                                actionLib.setShooterVel(1120),
                                new RaceAction(
                                        actionLib.setTurretPos(-50),
                                        new SleepAction(2)
                                ),
                                driveToPLShoot
                        ),
                        new SleepAction(2),
                        actionLib.shootAllBalls(),
                        new SleepAction(2),
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
                        new ParallelAction( // drive to first row & switch spindexer
                                driveToRow1,
                                actionLib.setIntakeVel(1400)
                                /*
                                new SequentialAction(
                                        new RaceAction(
                                                actionLib.setTurretPos(-110),
                                                new SleepAction(2)
                                        ),
                                        actionLib.scanMotif()
                                )
                                 */
                        ),
                        new RaceAction(
                                new ParallelAction(
                                        actionLib.scanColorToggle(),
                                        new SequentialAction(
                                                actionLib.spindexerTargetAddVel(20, 800),
                                                new SleepAction(1)
                                        )
                                ),
                                new SleepAction(3)
                        ),
                        actionLib.setIntakeVel(0),
                        actionLib.sortCurrentSpindexer(),
                        new SleepAction(2),
                        updatePose()
                )
        );

        Action driveToGate = drive.actionBuilder(latestPose)
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(openGate.position, Math.toRadians(90))
                // wait for gate
                .setTangent(Math.atan2(shoot1.position.y - openGate.position.y, shoot1.position.x - openGate.position.x))
                .lineToX(shoot1.position.x)
                                .build();

        // open gate
        Actions.runBlocking(
                new SequentialAction(
                        //actionLib.setIntakeVel(0),
                        driveToGate,
                        new SleepAction(1),
                        actionLib.shootAllBalls(),
                        new SleepAction(2),
                        updatePose()
                )
        );

        Action driveToRow2 = drive.actionBuilder(latestPose)
                .setTangent(0)
                .lineToX(preIntake2.position.x)
                // start intake
                .setTangent(Math.toRadians(90))
                .lineToY(postIntake2.position.y)
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        actionLib.setIntakeVel(1400),
                        driveToRow2,
                        new SleepAction(1),
                        actionLib.setIntakeVel(0),
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
                        driveToShoot2,
                        actionLib.shootAllBalls(),
                        new SleepAction(2),
                        updatePose()
                )
        );

        Action driveToRow3 = drive.actionBuilder(latestPose)
                .setTangent(0)
                .lineToX(preIntake3.position.x)
                // start intake
                .setTangent(Math.toRadians(90))
                .lineToY(postIntake3.position.y)
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        actionLib.setIntakeVel(1400),
                        driveToRow3,
                        new SleepAction(1),
                        actionLib.setIntakeVel(0),
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
                        driveToShoot3,
                        actionLib.shootAllBalls(),
                        new SleepAction(2),
                        driveOff
                )
        );
    }
}
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

@Autonomous(name="RR Blue SHOOTING", group="Alliance")
public class RRautoBlue extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    RobotHardware.RoadRunnerActions actionLib = robot.new RoadRunnerActions();
    Pose2d initialPose = new Pose2d(-48.5, -49.5, Math.toRadians(-126));
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

        Pose2d startPose = new Pose2d(-47, -50, Math.toRadians(-126));

        Pose2d shoot1 =  new Pose2d(-22, -16, Math.toRadians(-90));

        Pose2d preIntake1 = new Pose2d(-5, -22, Math.toRadians(-90));
        Pose2d postIntake1 = new Pose2d(preIntake1.position.x, -46, Math.toRadians(-92));

        Pose2d openGate = new Pose2d(5, -47, Math.toRadians(-90));

        Pose2d preIntake2 = new Pose2d(20.5, preIntake1.position.y, Math.toRadians(-90));
        Pose2d postIntake2 = new Pose2d(preIntake2.position.x, postIntake1.position.y-5.5, Math.toRadians(-90));

        Pose2d preIntake3 = new Pose2d(18, preIntake1.position.y, Math.toRadians(-90));
        Pose2d postIntake3 = new Pose2d(preIntake3.position.x, postIntake1.position.y-5, Math.toRadians(-100));

        // return to volley pose

        robot.setLocalizerPosition(startPose);

        Action driveToPLShoot = drive.actionBuilder(startPose)
                .setTangent(Math.atan2(shoot1.position.y - startPose.position.y, shoot1.position.x - startPose.position.x))
                //.lineToXLinearHeading(-28, Math.toRadians(180))
                .lineToXLinearHeading(shoot1.position.x, shoot1.heading)
                // scan & sort
                // shoot
                .build();

        // init done

        telemetry.clearAll();

        double angle = Math.toDegrees(-robot.turretAngleToTarget(new Vector2d(-65, -62), shoot1));

        waitForStart();

        if (isStopRequested()) return;

        robot.setMagManualBulk("GPP");

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                actionLib.setHoodAng(0.75),
                                actionLib.setShooterVel(1400),
                                new SequentialAction(
                                        new RaceAction(
                                                actionLib.setTurretPos(angle - 2),
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
                .lineToXConstantHeading(preIntake2.position.x)
                // start intake
                .setTangent(Math.toRadians(-90))
                .lineToYConstantHeading(postIntake2.position.y - 2, new TranslationalVelConstraint(90))

                .build();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction( // drive to first row & switch spindexer
                                driveToRow1,
                                actionLib.setIntakeVel(2600)
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
                        new SleepAction(0.2),
                        /*
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
                        actionLib.spindexerTargetAddVel(-20, 800),
                         */
                        //actionLib.sortCurrentSpindexer(),
                        //new SleepAction(2),
                        updatePose()
                )
        );

        Action driveToGate = drive.actionBuilder(latestPose)
                // stop intake
                .setTangent(Math.toRadians(90))
                .lineToYConstantHeading(postIntake2.position.y +7)

                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(openGate.position.x + 4.5, openGate.position.y), Math.toRadians(-90))
                .waitSeconds(0.75)


                .lineToYConstantHeading(openGate.position.y + 11)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(shoot1.position, Math.toRadians(180), new TranslationalVelConstraint(90))
                // shoot
                .build();

        // open gate
        Actions.runBlocking(
                new SequentialAction(
                        //actionLib.setIntakeVel(0),
                        new ParallelAction(
                                new SequentialAction(
                                        new RaceAction(
                                                actionLib.setTurretPos(angle - 2),
                                                new SleepAction(2)
                                        ),
                                        actionLib.stopTurretPower()
                                ),
                                driveToGate
                        ),
                        actionLib.setIntakeVel(0),
                        //new SleepAction(1),
                        actionLib.shootAllBalls(),
                        new SleepAction(1.5),
                        updatePose()
                )
        );

        Action driveToRow2 = drive.actionBuilder(latestPose)
                .setTangent(Math.toRadians(0))
                .lineToXConstantHeading(latestPose.position.x + 20)
                .splineToLinearHeading(new Pose2d(postIntake2.position.x + 0.75, openGate.position.y - 6, Math.toRadians(-105)),Math.toRadians(-90), new TranslationalVelConstraint(90))
                // intake from gate
                .waitSeconds(1)
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
                .setTangent(Math.toRadians(90))
                .lineToY(openGate.position.y +5)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(shoot1, Math.toRadians(180), new TranslationalVelConstraint(90))
                // shoot
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        driveToShoot2,
                        actionLib.setIntakeVel(0),
                        actionLib.shootAllBalls(),
                        new SleepAction(1.5),
                        updatePose()
                )
        );

        Action driveToRow3 = drive.actionBuilder(latestPose)
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(preIntake1.position.x+3, preIntake1.position.y), Math.toRadians(-90))
                //start intake
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(postIntake1.position.x + 3, postIntake1.position.y), Math.toRadians(-90),new TranslationalVelConstraint(90))
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
                .setTangent(Math.atan2(shoot1.position.y - postIntake1.position.y, shoot1.position.x - postIntake1.position.x))
                .lineToXConstantHeading(shoot1.position.x, new TranslationalVelConstraint(90))
                .build();

        Action driveOff = drive.actionBuilder(shoot1)
                .setTangent(0)
                .lineToXConstantHeading(preIntake2.position.x)
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        driveToShoot3,
                        actionLib.setIntakeVel(0),
                        actionLib.shootAllBalls(),
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
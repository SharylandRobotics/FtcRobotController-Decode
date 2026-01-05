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
        double intakeSpeed = 1;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(hardwareMap, initialPose);

        robot.init();

        // shoot pose PL 30 in back from startPose
        Pose2d shootVolleyPL = new Pose2d(-25, 17.3, initialPose.heading.real);

        Pose2d ballRow1 = new Pose2d(-6.5, 29, Math.toRadians(90));
        Pose2d ballRow1End = new Pose2d(ballRow1.position.x,46, Math.toRadians(90));

        Pose2d openGate = new Pose2d(ballRow1.position.x + 14, 58, Math.toRadians(90));

        Pose2d shootVolleyPose = new Pose2d(ballRow1.position.x,23, Math.toRadians(90));

        Pose2d ballRow2 = new Pose2d(24, 33, Math.toRadians(88));
        Pose2d ballRow2End = new Pose2d(ballRow2.position.x, 48, Math.toRadians(90));

        Pose2d ballRow3 = new Pose2d(51, 33, Math.toRadians(85));
        Pose2d ballRow3End = new Pose2d(ballRow3.position.x, 48, Math.toRadians(90));

        // return to volley pose



        Action driveToPLShoot = drive.actionBuilder(initialPose)
                .setTangent(Math.atan2(shootVolleyPL.position.y - initialPose.position.y,
                        shootVolleyPL.position.x - initialPose.position.x))
                .lineToY(shootVolleyPL.position.y)
                // scan & sort
                // shoot
                .build();

        // init done

        Actions.runBlocking(
                new SequentialAction(
                        actionLib.setHoodAng(0.4)
                )
        );

        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        robot.setMagManualBulk("GPP");

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                actionLib.setShooterVel(1120),
                                new RaceAction(
                                    actionLib.setTurretPos(-10),
                                        new SleepAction(1.6)
                                ),
                                driveToPLShoot
                        )

                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        actionLib.shootAllBalls(), // shoot first 3
                        updatePose(),
                        new SleepAction(2)
                )
        );

        Action driveToRow1 = drive.actionBuilder(latestPose)
                .setTangent(Math.atan2(ballRow1.position.y - latestPose.position.y,
                        ballRow1.position.x - latestPose.position.x))
                .lineToYLinearHeading(ballRow1.position.y, ballRow1.heading)
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction( // drive to first row & switch spindexer
                                driveToRow1,
                                actionLib.setIntakeVel(intakeSpeed),
                                new SequentialAction(
                                        new RaceAction(
                                                actionLib.setTurretPos(-110),
                                                new SleepAction(2)
                                        ),
                                        actionLib.scanMotif()
                                )
                        ),

                        updatePose()
                )
        );

        Action driveToRow1End = drive.actionBuilder(latestPose)
                .setTangent(Math.atan2(ballRow1End.position.y - latestPose.position.y,
                        ballRow1End.position.x - latestPose.position.x))
                .lineToY(ballRow1End.position.y, new TranslationalVelConstraint(10))
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                driveToRow1End,
                                new SequentialAction(

                                        new SleepAction(0.7),
                                        actionLib.spindexerTargetAddVel(120, 1200),
                                        new SleepAction(0.38),
                                        actionLib.spindexerTargetAddVel(120, 1200)


                                ),
                                new SequentialAction(
                                        new RaceAction(
                                                actionLib.setTurretPos(-50),
                                                new SleepAction(1.6)

                                        ),
                                        actionLib.stopTurretPower()
                                )
                        ),

                        actionLib.setIntakeVel(0), // finish intaking 1st row
                        //limelight.setMagBatch("GPP"),

                        updatePose()
                )
        );

        robot.setMagManualBulk("GPP");

        Action driveToGate = drive.actionBuilder(latestPose)
                .setTangent(-90)
                        .splineToConstantHeading(new Vector2d(openGate.position.x, openGate.position.y-10), Math.toRadians(180))
                .setTangent(Math.toRadians(90))
                .lineToY(openGate.position.y)
                                .build();

        // open gate
        Actions.runBlocking(
                new SequentialAction(
                        //actionLib.setIntakeVel(0),
                        driveToGate,
                        updatePose()
                )
        );

        Action driveToVolleyPose = drive.actionBuilder(latestPose)
                .setTangent(Math.atan2(shootVolleyPose.position.y - latestPose.position.y,
                        shootVolleyPose.position.x - latestPose.position.x))
                .lineToY(shootVolleyPose.position.y)
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        actionLib.setIntakeVel(intakeSpeed),
                        new ParallelAction(
                                actionLib.setShooterVel(1220),
                                driveToVolleyPose,
                                actionLib.sortCurrentSpindexer()
                                /*
                                new RaceAction(
                                        actionLib.setTurretPos(-43),
                                        new SleepAction(1.5)
                                )

                                 */
                        ),
                        //actionLib.stopTurretPower(),
                        actionLib.shootAllBalls(),
                        updatePose(),
                        new SleepAction(2)
                )
        );

        // next row

        Action driveToRow2 = drive.actionBuilder(latestPose)
                .setTangent(Math.atan2(ballRow2.position.y - latestPose.position.y,
                        ballRow2.position.x - latestPose.position.x))
                .lineToYLinearHeading(ballRow2.position.y, ballRow2.heading)
                .build();

        Actions.runBlocking(
                new SequentialAction(

                        new ParallelAction( // drive to 2nd row & switch spindexer
                                driveToRow2,
                                actionLib.setIntakeVel(intakeSpeed)
                        ),

                        updatePose()
                )
        );

        Action driveToRow2End = drive.actionBuilder(latestPose)
                .setTangent(Math.atan2(ballRow2End.position.y - latestPose.position.y,
                        ballRow2End.position.x - latestPose.position.x))
                .lineToY(ballRow2End.position.y, new TranslationalVelConstraint(10))
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                driveToRow2End,
                                new SequentialAction(

                                        new SleepAction(0.7),
                                        actionLib.spindexerTargetAddVel(120, 1200),
                                        new SleepAction(0.38),
                                        actionLib.spindexerTargetAddVel(120, 1200)


                                )
                        ),

                        actionLib.setIntakeVel(0), // finish intaking 1st row
                        //limelight.setMagBatch("PGP"),

                        updatePose()
                )
        );


        Action driveToVolleyPose2 = drive.actionBuilder(latestPose)
                .setTangent(Math.atan2(shootVolleyPose.position.y - latestPose.position.y,
                        shootVolleyPose.position.x - latestPose.position.x))
                .lineToY(shootVolleyPose.position.y)
                .build();

        telemetry.update();

        robot.setMagManualBulk("PGP");

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                driveToVolleyPose2,
                                new SequentialAction(
                                        actionLib.sortCurrentSpindexer()
                                        //spindexer.spindexerTargetAdd(-120)
                                )
                        ),

                        new SleepAction(0.6),
                        actionLib.shootAllBalls(),
                        updatePose(),
                        new SleepAction(2)
                )
        );

        Action driveToRow3 = drive.actionBuilder(latestPose)
                .setTangent(Math.atan2(ballRow3.position.y - latestPose.position.y,
                        ballRow3.position.x - latestPose.position.x))
                .lineToYLinearHeading(ballRow3.position.y, ballRow3.heading)
                .build();



        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                driveToRow3,
                                actionLib.setIntakeVel(intakeSpeed)
                        ),
                        updatePose()
                )
        );

        Action driveToRow3End = drive.actionBuilder(latestPose)
                .setTangent(Math.atan2(ballRow3End.position.y - latestPose.position.y,
                        ballRow3End.position.x - latestPose.position.x))
                .lineToY(ballRow3End.position.y, new TranslationalVelConstraint(10))
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                driveToRow3End,
                                new SequentialAction(

                                        new SleepAction(0.7),
                                        actionLib.spindexerTargetAddVel(120, 1200),
                                        new SleepAction(0.38),
                                        actionLib.spindexerTargetAddVel(120, 1200)


                                )
                        ),

                        actionLib.setIntakeVel(0), // finish intaking 1st row
                        //limelight.setMagBatch("PGP"),

                        updatePose()
                )
        );

        Action driveToVolleyPose3 = drive.actionBuilder(latestPose)
                .setTangent(Math.atan2(shootVolleyPose.position.y - latestPose.position.y,
                        shootVolleyPose.position.x - latestPose.position.x))
                .lineToY(shootVolleyPose.position.y)
                .build();

        robot.setMagManualBulk("PPG");

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                driveToVolleyPose3,
                                actionLib.sortCurrentSpindexer()
                        ),

                        new SleepAction(0.6),
                        actionLib.shootAllBalls(),
                        updatePose(),
                        new SleepAction(2)
                )
        );
    }
}
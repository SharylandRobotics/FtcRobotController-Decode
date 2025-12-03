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

@Autonomous(name="RR Auto Red", group="Alliance")
public class RRautoRed extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    RobotHardware.RoadRunnerActions actionLib = robot.new RoadRunnerActions();
    Pose2d initialPose = new Pose2d(-48.5, 49.5, Math.toRadians(125));
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
        drive = new MecanumDrive(hardwareMap, initialPose);

        robot.init();

        // shoot pose PL 30 in back from startPose
        Pose2d shootVolleyPL = new Pose2d(-25, 17.3, initialPose.heading.real);

        Pose2d ballRow1 = new Pose2d(-6.5, 29, Math.toRadians(90));
        Pose2d ballRow1End = new Pose2d(ballRow1.position.x,44.2, Math.toRadians(90));

        Pose2d openGate = new Pose2d(ballRow1.position.x + 8, 52, Math.toRadians(90));

        Pose2d shootVolleyPose = new Pose2d(ballRow1.position.x,23, Math.toRadians(90));

        Pose2d ballRow2 = new Pose2d(23, 30, Math.toRadians(90));
        Pose2d ballRow2End = new Pose2d(ballRow2.position.x, 44.2, Math.toRadians(90));

        Pose2d ballRow3 = new Pose2d(48, 29, Math.toRadians(90));
        Pose2d ballRow3End = new Pose2d(ballRow3.position.x, 44.2, Math.toRadians(90));

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
                                actionLib.setShooterVel((int) (800*1.4)),
                                new SequentialAction(
                                        new ParallelAction(
                                                new RaceAction(
                                                        actionLib.setTurretPos(-70),
                                                        new SleepAction(2)
                                                ),
                                                new SequentialAction(
                                                        new SleepAction(0.6),
                                                        actionLib.scanMotif()
                                                )
                                        ),
                                        new RaceAction(
                                                actionLib.setTurretPos(-5),
                                                new SleepAction(1.6)

                                        ),
                                        actionLib.stopTurretPower()
                                ),
                                driveToPLShoot
                        )

                )
        );

        int pattern = robot.solvePattern()[0];

        Actions.runBlocking(
                new SequentialAction(
                        actionLib.sortCurrentSpindexer(),

                        new SleepAction(1),
                        actionLib.shootAllBalls(), // shoot first 3
                        updatePose(),
                        new SleepAction(3)
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
                                actionLib.setIntakeVel(-1400)
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
                                        actionLib.spindexerTargetAddVel(120, 1000),
                                        new SleepAction(0.38),
                                        actionLib.spindexerTargetAddVel(120, 1000)


                                )
                        ),

                        actionLib.setIntakeVel(0), // finish intaking 1st row
                        //limelight.setMagBatch("GPP"),
                        new SleepAction(1),

                        updatePose()
                )
        );

        robot.setMagManualBulk("GPP");

        Action driveToGate = drive.actionBuilder(latestPose)
                .setTangent(-90)
                        .splineToConstantHeading(openGate.position, Math.toRadians(90))
                                .build();

        // open gate
        Actions.runBlocking(
                new SequentialAction(
                        driveToGate,
                        updatePose()
                )
        );

        Action driveToVolleyPose = drive.actionBuilder(latestPose)
                .setTangent(Math.atan2(shootVolleyPose.position.y - latestPose.position.y,
                        shootVolleyPose.position.x - latestPose.position.x))
                .lineToY(shootVolleyPose.position.y)
                .build();

        pattern = robot.solvePattern()[0];
        telemetry.addData("pattern: ", RobotHardware.pattern);
        telemetry.addData("turn: ", pattern);
        telemetry.update();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                driveToVolleyPose,
                                actionLib.sortCurrentSpindexer(),
                                new RaceAction(
                                        actionLib.setTurretPos(-45),
                                        new SleepAction(1.5)
                                )
                        ),
                        actionLib.stopTurretPower(),
                        actionLib.shootAllBalls(),
                        updatePose(),
                        new SleepAction(3)
                )
        );

        // next row

        Action driveToRow2 = drive.actionBuilder(latestPose)
                .setTangent(Math.atan2(ballRow2.position.y - latestPose.position.y,
                        ballRow2.position.x - latestPose.position.x))
                .lineToY(ballRow2.position.y)
                .build();

        Actions.runBlocking(
                new SequentialAction(

                        new ParallelAction( // drive to 2nd row & switch spindexer
                                driveToRow2,
                                actionLib.setIntakeVel(-1400)
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
                                        actionLib.spindexerTargetAddVel(120, 1000),
                                        new SleepAction(0.38),
                                        actionLib.spindexerTargetAddVel(120, 1000)


                                )
                        ),

                        actionLib.setIntakeVel(0), // finish intaking 1st row
                        //limelight.setMagBatch("PGP"),
                        new SleepAction(1),

                        updatePose()
                )
        );


        Action driveToVolleyPose2 = drive.actionBuilder(latestPose)
                .setTangent(Math.atan2(shootVolleyPose.position.y - latestPose.position.y,
                        shootVolleyPose.position.x - latestPose.position.x))
                .lineToY(shootVolleyPose.position.y)
                .build();

        pattern = robot.solvePattern()[0];
        telemetry.addData("turn: ", pattern);
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
                        new SleepAction(3)
                )
        );

        Action driveToRow3 = drive.actionBuilder(latestPose)
                .setTangent(Math.atan2(ballRow3.position.y - latestPose.position.y,
                        ballRow3.position.x - latestPose.position.x))
                .lineToY(ballRow3.position.y)
                .build();



        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                driveToRow3,
                                actionLib.setIntakeVel(-1400)
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
                                        actionLib.spindexerTargetAddVel(120, 1000),
                                        new SleepAction(0.38),
                                        actionLib.spindexerTargetAddVel(120, 1000)


                                )
                        ),

                        actionLib.setIntakeVel(0), // finish intaking 1st row
                        //limelight.setMagBatch("PGP"),
                        new SleepAction(1),

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
                                new SequentialAction(
                                        actionLib.sortCurrentSpindexer()
                                        //spindexer.spindexerTargetAdd(-120)
                                )
                        ),

                        new SleepAction(0.6),
                        actionLib.shootAllBalls(),
                        updatePose(),
                        new SleepAction(3)
                )
        );
    }
}
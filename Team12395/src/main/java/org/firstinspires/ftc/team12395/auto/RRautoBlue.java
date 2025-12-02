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

@Autonomous(name="RR Auto Blue", group="Alliance")
public class RRautoBlue extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    RobotHardware.RoadRunnerActions actionLib = robot.new RoadRunnerActions();
    Pose2d initialPose = new Pose2d(-38, -54, Math.toRadians(-90));
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

        // BLUE
        // shoot pose PL 30 in back from startPose
        Pose2d shootVolleyPL = new Pose2d(-23, -23, initialPose.heading.real);

        Pose2d ballRow1 = new Pose2d(-11.5, -30, initialPose.heading.real);
        Pose2d ballRow1End = new Pose2d(ballRow1.position.x,-43,initialPose.heading.real);

        Pose2d openGate = new Pose2d(-3,-52, initialPose.heading.real);

        Pose2d shootVolleyPose = new Pose2d(-10,-22, initialPose.heading.real);

        Pose2d ballRow2 = new Pose2d(11.5, -35, initialPose.heading.real);
        Pose2d ballRow2End = new Pose2d(ballRow2.position.x, -43, initialPose.heading.real);

        Pose2d ballRow3 = new Pose2d(35, -38, initialPose.heading.real);
        Pose2d ballRow3End = new Pose2d(ballRow3.position.x, -43, initialPose.heading.real);

        Action driveToPL = drive.actionBuilder(initialPose)
                .setTangent(Math.atan2(shootVolleyPL.position.y - initialPose.position.y,
                        shootVolleyPL.position.x - initialPose.position.x))
                .lineToY(shootVolleyPL.position.y)
                        .build();

        // init action

        Actions.runBlocking(
                new SequentialAction(
                        actionLib.setHoodAng(0)
                )
        );

        waitForStart();



        Actions.runBlocking(
                new SequentialAction(
                    actionLib.setShooterVel(1120),
                    new ParallelAction(
                        new RaceAction(
                                actionLib.setTurretPos(60),
                                new SleepAction(10)
                        ),
                        driveToPL
                    ),
                    actionLib.shootAllBalls(),
                    updatePose()

                )
        );

        Action driveToRow1 = drive.actionBuilder(latestPose)
                .setTangent(Math.toRadians(-20))
                .splineToConstantHeading(ballRow1.position, Math.toRadians(-90))

                .lineToY(ballRow1End.position.y, new TranslationalVelConstraint(10))
                .splineToSplineHeading(openGate, Math.toRadians(60))
                .splineToConstantHeading(shootVolleyPose.position, Math.toRadians(90))

                .build();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                new SequentialAction(
                                    new RaceAction(
                                            actionLib.setTurretPos(110),
                                            actionLib.scanMotif()
                                    ),
                                    new RaceAction(
                                            actionLib.setTurretPos(60),
                                            new SleepAction(10)
                                    )
                                ),
                                driveToRow1,
                                actionLib.setIntakeVel(-1400),
                                new SequentialAction(
                                        new RaceAction(
                                                new SleepAction(15),
                                                actionLib.automaticallyIntakeBalls()
                                        ),
                                        actionLib.sortCurrentSpindexer()
                                )
                        ),
                        actionLib.shootAllBalls(),
                        updatePose()
                )
        );

        Action driveToRow2 = drive.actionBuilder(latestPose)
                .setTangent(Math.atan2(ballRow2.position.y - latestPose.position.y,
                        ballRow2.position.x - latestPose.position.x))
                .splineToConstantHeading(ballRow2.position, Math.toRadians(-90))

                .lineToY(ballRow2End.position.y)
                .splineToSplineHeading(shootVolleyPose, Math.toRadians(135))

                .build();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                driveToRow2,
                                new SequentialAction(
                                        new RaceAction(
                                                new SleepAction(15),
                                                actionLib.automaticallyIntakeBalls()
                                        ),
                                        actionLib.sortCurrentSpindexer()
                                )
                        ),
                        actionLib.shootAllBalls(),
                        updatePose()
                )
        );

        Action driveToRow3 = drive.actionBuilder(latestPose)
                .setTangent(Math.atan2(ballRow3.position.y -latestPose.position.y,
                        ballRow3.position.x - latestPose.position.x))
                .splineToConstantHeading(ballRow3.position, Math.toRadians(-90))

                .lineToY(ballRow3End.position.y)
                .splineToSplineHeading(shootVolleyPose, Math.toRadians(135))

                .build();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                driveToRow3,
                                new SequentialAction(
                                        new RaceAction(
                                                new SleepAction(15),
                                                actionLib.automaticallyIntakeBalls()
                                        ),
                                        actionLib.sortCurrentSpindexer()
                                )
                        ),
                        actionLib.shootAllBalls(),
                        updatePose()
                )
        );

        Action park = drive.actionBuilder(latestPose)
                        .lineToX(0)
                                .build();

        Actions.runBlocking(
                new SequentialAction(
                        park
                )
        );
    }
}
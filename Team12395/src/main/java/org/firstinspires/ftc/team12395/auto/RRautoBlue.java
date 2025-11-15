package org.firstinspires.ftc.team12395.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team12395.rr.MecanumDrive;
import org.firstinspires.ftc.team12395.rrActions;
import org.firstinspires.ftc.team12395.RobotHardware;

import java.lang.Math;

@Autonomous(name="RR Auto Blue", group="Alliance")
public class RRautoBlue extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    rrActions actionLib = new rrActions(robot, this);
    Pose2d initialPose = new Pose2d(-48.5, -49.5, Math.toRadians(-125));
    MecanumDrive drive;
    Pose2d latestPose = initialPose;

    private Action updatePose(){
        return (TelemetryPacket packet) -> {
            drive.updatePoseEstimate();
            latestPose = drive.localizer.getPose();
            return false;
        };
    }
    rrActions.Hood hood = actionLib.new Hood();
    rrActions.Intake intake = actionLib.new Intake();
    rrActions.liftArm liftArm = actionLib.new liftArm();
    rrActions.LimeLight limelight = actionLib.new LimeLight();
    rrActions.Shooter shooter = actionLib.new Shooter();
    rrActions.Spindexer spindexer = actionLib.new Spindexer();
    rrActions.Turret turret = actionLib.new Turret();

    public Action shoot3() {
        return new SequentialAction(
                liftArm.liftArmUp(),
                new SleepAction(0.75),
                liftArm.liftArmDown(),
                new SleepAction(0.75),
                spindexer.spindexerTargetAdd(-120),

                new SleepAction(0.75),

                liftArm.liftArmUp(),
                new SleepAction(0.75),
                liftArm.liftArmDown(),
                new SleepAction(0.75),
                spindexer.spindexerTargetAdd(-120),

                new SleepAction(0.75),

                liftArm.liftArmUp(),
                new SleepAction(0.75),
                liftArm.liftArmDown()
        );
    }


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(hardwareMap, initialPose);

        robot.init();



        // shoot pose PL 30 in back from startPose
        Pose2d shootVolleyPL = new Pose2d(-25, -17.3, initialPose.heading.real);

        Pose2d ballRow1 = new Pose2d(-6.5, -29, Math.toRadians(-90));
        Pose2d ballRow1End = new Pose2d(ballRow1.position.x,-44.2, Math.toRadians(-90));

        Pose2d shootVolleyPose = new Pose2d(ballRow1.position.x,-23, Math.toRadians(-90));

        Pose2d ballRow2 = new Pose2d(22.5, -30, Math.toRadians(-90));
        Pose2d ballRow2End = new Pose2d(ballRow2.position.x, -44.2, Math.toRadians(-90));

        Pose2d ballRow3 = new Pose2d(35.5, -29, Math.toRadians(-90));

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
                        hood.setHoodAngle(0.4)
                )
        );

        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                shooter.setShooterVel((int) (800*1.4)),
                                new SequentialAction(
                                    new ParallelAction(
                                            new RaceAction(
                                                    turret.turnTurretTo(70),
                                                    new SleepAction(2)
                                            ),
                                            new SequentialAction(
                                                 new SleepAction(0.5),
                                                 limelight.scanForObelisk()
                                            )
                                    ),
                                    spindexer.sortSpindexer(),
                                    new RaceAction(
                                           turret.turnTurretTo(5),
                                           new SleepAction(1.5)
                                    )
                                ),
                                driveToPLShoot
                        ),
                        new SleepAction(1),
                        shoot3(), // shoot first 3
                        updatePose()
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
                                spindexer.spindexerTargetAdd(60),
                                intake.setIntakeVel(-1400)
                        ),

                        updatePose()
                )
        );

        Action driveToRow1End = drive.actionBuilder(latestPose)
                .setTangent(Math.atan2(ballRow1End.position.y - latestPose.position.y,
                        ballRow1End.position.x - latestPose.position.x))
                .lineToY(ballRow1End.position.y, new TranslationalVelConstraint(10.5))
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                            driveToRow1End,
                            new SequentialAction(
                                    new SleepAction(0.7),
                                    spindexer.spindexerTargetAddVel(120, 1000),
                                    new SleepAction(0.38),
                                    spindexer.spindexerTargetAddVel(120, 1000)
                            )
                        ),

                        intake.setIntakeVel(0), // finish intaking 1st row
                        limelight.setMagBatch("GPP"),
                        spindexer.spindexerTargetAdd(60),

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
                        new ParallelAction(
                            driveToVolleyPose,
                            spindexer.sortSpindexer(),
                            new RaceAction(
                                    turret.turnTurretTo(45),
                                    new SleepAction(1.5)
                            )
                        ),

                        new SleepAction(1),
                        shoot3(),

                        updatePose()
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
                                spindexer.spindexerTargetAdd(60),
                                intake.setIntakeVel(-1400)
                        ),

                        updatePose()
                )
        );

        Action driveToRow2End = drive.actionBuilder(latestPose)
                .setTangent(Math.atan2(ballRow2End.position.y - latestPose.position.y,
                        ballRow2End.position.x - latestPose.position.x))
                .lineToY(ballRow2End.position.y, new TranslationalVelConstraint(10.5))
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                driveToRow2End,
                                new SequentialAction(
                                        new SleepAction(0.7),
                                        spindexer.spindexerTargetAddVel(-120, 1000),
                                        new SleepAction(0.38),
                                        spindexer.spindexerTargetAddVel(-120, 1000)
                                )
                        ),

                        intake.setIntakeVel(0), // finish intaking 1st row
                        limelight.setMagBatch("PGP"),
                        spindexer.spindexerTargetAdd(60),

                        updatePose()
                )
        );


        Action driveToVolleyPose2 = drive.actionBuilder(latestPose)
                .setTangent(Math.atan2(shootVolleyPose.position.y - latestPose.position.y,
                        shootVolleyPose.position.x - latestPose.position.x))
                .lineToY(shootVolleyPose.position.y)
                .build();


        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                            driveToVolleyPose2,
                            spindexer.sortSpindexer()
                        ),

                        new SleepAction(1),
                        shoot3(),

                        updatePose()
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
                                shooter.setShooterVel(0),
                                turret.turnTurretTo(0),
                                spindexer.spindexerTargetAdd(60),
                                hood.setHoodAngle(1)
                        )
                )
        );
    }
}
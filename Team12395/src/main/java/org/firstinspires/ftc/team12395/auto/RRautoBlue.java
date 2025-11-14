package org.firstinspires.ftc.team12395.auto;

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
    Pose2d initialPose = new Pose2d(-48, -50, Math.toRadians(-125));
    MecanumDrive drive;
    Pose2d latestPose = initialPose;

    private Action updatePose(){
        return (TelemetryPacket packet) -> {
            drive.updatePoseEstimate();
            latestPose = drive.localizer.getPose();
            return false;
        };
    }

    private Action showMarker(String marker){
        return (TelemetryPacket packet) -> {
            packet.put(marker, "");
            return false;
        };
    }

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        robot.init();

        rrActions.Hood hood = actionLib.new Hood();
        rrActions.Intake intake = actionLib.new Intake();
        rrActions.liftArm liftArm = actionLib.new liftArm();
        rrActions.LimeLight limelight = actionLib.new LimeLight();
        rrActions.Shooter shooter = actionLib.new Shooter();
        rrActions.Spindexer spindexer = actionLib.new Spindexer();
        rrActions.Turret turret = actionLib.new Turret();

        // shoot pose PL 30 in back from startPose
        Pose2d shootVolleyPL = new Pose2d(-25, -17.3, initialPose.heading.real);

        Pose2d ballRow1 = new Pose2d(-12, -29, Math.toRadians(-90));
        Pose2d ballRow1End = new Pose2d(-12,-44.2, Math.toRadians(-90));

        Pose2d shootVolleyPose = new Pose2d(-12,-23, Math.toRadians(-90));

        Pose2d ballRow2 = new Pose2d(12, -29, Math.toRadians(-90));
        Pose2d ballRow2End = new Pose2d(12, -44.2, Math.toRadians(-90));

        Pose2d ballRow3 = new Pose2d(34.5, -29, Math.toRadians(-90));

        // return to volley pose

        Action shoot3 = new SequentialAction(

                liftArm.liftArmUp(),
                new SleepAction(0.75),
                liftArm.liftArmDown(),
                new SleepAction(0.75),
                spindexer.spindexerTargetAdd(120),

                new SleepAction(0.75),

                liftArm.liftArmUp(),
                new SleepAction(0.75),
                liftArm.liftArmDown(),
                new SleepAction(0.75),
                spindexer.spindexerTargetAdd(120),

                new SleepAction(0.75),

                liftArm.liftArmUp(),
                new SleepAction(0.75),
                liftArm.liftArmDown()
        );

        Action driveToPLShoot = drive.actionBuilder(initialPose)
                .setTangent(Math.atan2(shootVolleyPL.position.y - initialPose.position.y,
                        shootVolleyPL.position.x - initialPose.position.x))
                .lineToY(shootVolleyPL.position.y)
                // scan & sort
                // shoot
                .build();

        Action driveToRow1 = drive.actionBuilder(latestPose)
                .setTangent(Math.atan2(ballRow1.position.y - latestPose.position.y,
                        ballRow1.position.x - latestPose.position.x))
                .lineToY(ballRow1.position.y)
                .build();

        Action driveToRow1End = drive.actionBuilder(latestPose)
                .setTangent(Math.atan2(ballRow1End.position.y - latestPose.position.y,
                        ballRow1End.position.x - latestPose.position.x))
                .lineToY(ballRow1.position.y, new TranslationalVelConstraint(10))
                .afterDisp(3, spindexer.spindexerTargetAdd(120))
                .afterDisp(8, spindexer.spindexerTargetAdd(120))
                .afterDisp(13, spindexer.spindexerTargetAdd(120))
                .build();

        Action driveToVolleyPose = drive.actionBuilder(latestPose)
                .setTangent(Math.atan2(shootVolleyPose.position.y - latestPose.position.y,
                        shootVolleyPose.position.x - latestPose.position.x))
                .lineToY(shootVolleyPose.position.y)
                .build();

        Action driveToRow2 = drive.actionBuilder(latestPose)
                .setTangent(Math.atan2(ballRow2.position.y - latestPose.position.y,
                        ballRow2.position.x - latestPose.position.x))
                .lineToY(ballRow2.position.y)
                .build();

        Action driveToRow2End = drive.actionBuilder(latestPose)
                .setTangent(Math.atan2(ballRow2End.position.y - latestPose.position.y,
                        ballRow2End.position.x - latestPose.position.x))
                .lineToY(ballRow2.position.y, new TranslationalVelConstraint(10))
                .afterDisp(3, spindexer.spindexerTargetAdd(120))
                .afterDisp(8, spindexer.spindexerTargetAdd(120))
                .afterDisp(13, spindexer.spindexerTargetAdd(120))
                .build();

        Action driveToRow3 = drive.actionBuilder(latestPose)
                .setTangent(Math.atan2(ballRow3.position.y - latestPose.position.y,
                        ballRow3.position.x - latestPose.position.x))
                .lineToY(ballRow3.position.y)
                .build();


        // init done

        Actions.runBlocking(
                new SequentialAction(
                        hood.setHoodAngle(0.4)
                )
        );

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                shooter.setShooterVel((int) (800*1.4)),

                                new SequentialAction(
                                    new RaceAction(
                                        limelight.scanForObelisk(),
                                        new SequentialAction(
                                            turret.turnTurretTo(70),
                                            new SleepAction(10)
                                        )
                                    ),
                                    spindexer.sortSpindexer(),
                                    turret.turnTurretTo(0)
                                ),
                                driveToPLShoot

                        ),
                        limelight.scanForBlue(),
                        turret.turnTurretByGoalErr(),
                        new SleepAction(2),
                        shoot3, // shoot first 3

                        new SleepAction(2),
                        updatePose(),

                        new ParallelAction( // drive to first row & switch spindexer
                                driveToRow1,
                                spindexer.spindexerTargetAdd(60),
                                intake.setIntakeVel(1000)
                        ),

                        updatePose(),

                        driveToRow1End,
                        intake.setIntakeVel(0), // finish intaking 1st row
                        limelight.setMagBatch("GPP"),
                        spindexer.spindexerTargetAdd(60)

                        /*

                        updatePose(),
                        new SleepAction(1),
                        new ParallelAction(
                            driveToVolleyPose,
                            spindexer.sortSpindexer()
                        ),

                        limelight.scanForBlue(),
                        turret.turnTurretByGoalErr(),

                        new SleepAction(2),
                        shoot3,

                        updatePose(),
                        new SleepAction(2),

                        new ParallelAction( // drive to 2nd row & switch spindexer
                                driveToRow2,
                                spindexer.spindexerTargetAdd(60),
                                intake.setIntakeVel(1000)
                        ),

                        updatePose(),

                        driveToRow2End,
                        intake.setIntakeVel(0), // finish intaking 2nd row
                        limelight.setMagBatch("PGP"),
                        spindexer.spindexerTargetAdd(60),

                        updatePose(),
                        new SleepAction(1),
                        new ParallelAction(
                            driveToVolleyPose,
                            spindexer.sortSpindexer()
                        ),

                        limelight.scanForBlue(),
                        turret.turnTurretByGoalErr(),

                        new SleepAction(2),
                        shoot3,

                        updatePose(),
                        new ParallelAction(
                                driveToRow3,
                                shooter.setShooterVel(0),
                                turret.turnTurretTo(0),
                                spindexer.spindexerTargetAdd(60),
                                hood.setHoodAngle(1)
                        )

                        */
                )
        );
    }
}
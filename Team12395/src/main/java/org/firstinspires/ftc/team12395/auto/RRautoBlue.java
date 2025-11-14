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

        Pose2d ballRow1 = new Pose2d(-12, -28.1, Math.toRadians(-90));
        Pose2d ballRow1End = new Pose2d(-12,-43.1, Math.toRadians(-90));

        Pose2d shootVolleyPose = new Pose2d(-12,-23, Math.toRadians(-90));

        Pose2d ballRow2 = new Pose2d(12, -28.1, Math.toRadians(-90));
        Pose2d ballRow2End = new Pose2d(12, -43.1, Math.toRadians(-90));

        Pose2d pushCurvePose2 = new Pose2d(58, -18, -Math.PI/2);
        Pose2d pushEndPose2 = new Pose2d(58,-50, -Math.PI /2);
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
                        showMarker("DONE DRIVING"),
                        limelight.scanForBlue(),
                        showMarker("DONE SCANNING"),
                        turret.turnTurretByGoalErr(),
                        showMarker("DONE AIMING"),
                        new SleepAction(2),


                        shoot3,
                        new SleepAction(2),
                        updatePose(),
                        new ParallelAction(
                                driveToRow1,
                                spindexer.spindexerTargetAdd(60),
                                new SequentialAction(
                                        new SleepAction(1),
                                        intake.setIntakeVel(1000)
                                )
                        ),

                        updatePose(),
                        new ParallelAction(
                                driveToRow1End,
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        spindexer.spindexerTargetAdd(120)
                                )
                        )







                )
        );

    }
    private Action sleepAction(long milliseconds) {
        return (TelemetryPacket packet) -> {
            sleep(milliseconds);
            return false;
        };
    }
}
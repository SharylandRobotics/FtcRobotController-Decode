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

@Autonomous(name="RR SHORT Auto Red", group="Alliance")
public class RRautoRedCut extends LinearOpMode {

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

        Pose2d startPose = new Pose2d(-47, 50, Math.toRadians(90));

        Pose2d shoot1 =  new Pose2d(-22, 16, Math.toRadians(90));

        Pose2d preIntake1 = new Pose2d(-8, 22, Math.toRadians(90));
        Pose2d postIntake1 = new Pose2d(preIntake1.position.x, 42, Math.toRadians(90));

        Pose2d openGate = new Pose2d(4, postIntake1.position.y, Math.toRadians(90));

        Pose2d preIntake2 = new Pose2d(15, preIntake1.position.y, Math.toRadians(90));
        Pose2d postIntake2 = new Pose2d(preIntake2.position.x, postIntake1.position.y+2, Math.toRadians(90));

        Pose2d preIntake3 = new Pose2d(39, preIntake1.position.y, Math.toRadians(90));
        Pose2d postIntake3 = new Pose2d(preIntake3.position.x, postIntake1.position.y+2, Math.toRadians(90));

        // return to volley pose

        robot.setLocalizerPosition(startPose);

        Action driveToPLShoot = drive.actionBuilder(startPose)
                .setTangent(0)
                .lineToY(startPose.position.y + 18)
                .build();

        // init done

        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        robot.setMagManualBulk("GPP");

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                actionLib.setHoodAng(0.4),
                                new RaceAction(
                                        actionLib.setTurretPos(-40),
                                        new SleepAction(2)
                                ),
                                actionLib.setShooterVel(2000)
                        ),
                        new SleepAction(2),
                        actionLib.shootAllBalls(),
                        new SleepAction(1.5),
                        driveToPLShoot,
                        new RaceAction(
                                actionLib.setTurretPos(0),
                                new SleepAction(2)
                        )
                )
        );
    }
}
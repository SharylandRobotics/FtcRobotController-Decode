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

@Autonomous(name="RR color Sort", group="Alliance")
public class RRcolorSort extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    RobotHardware.RoadRunnerActions actionLib = robot.new RoadRunnerActions();
    Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
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

        Pose2d startPose = initialPose;

        Pose2d shoot1 =  new Pose2d(0, 20, Math.toRadians(90));

        // return to volley pose

        robot.setLocalizerPosition(startPose);

        Action driveToBalls = drive.actionBuilder(startPose)
                .setTangent(Math.toRadians(90))
                .lineToY(shoot1.position.y)
                .build();

        // init done

        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        robot.setMagManualBulk("000");

        Actions.runBlocking(
                new SequentialAction(
                        actionLib.setIntakeVel(2400),
                        new SleepAction(2),
                        new RaceAction(
                                actionLib.scanColorToggle(),
                                new SequentialAction(
                                        actionLib.spindexerTargetAddVel(20,800),
                                        new SleepAction(2)
                                )
                        ),
                        new SleepAction(2),
                        actionLib.sortCurrentSpindexer(),
                        new SleepAction(3)
                )
        );
    }
}
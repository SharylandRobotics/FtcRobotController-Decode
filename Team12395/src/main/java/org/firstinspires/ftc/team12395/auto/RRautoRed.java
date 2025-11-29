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

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(hardwareMap, initialPose);

        robot.init();

        // BLUE
        // shoot pose PL 30 in back from startPose
        Pose2d shootVolleyPL = new Pose2d(-25, -17.3, initialPose.heading.real);

        Pose2d ballRow1 = new Pose2d(-6.5, -29, Math.toRadians(-90));
        Pose2d ballRow1End = new Pose2d(ballRow1.position.x,-44.2, Math.toRadians(-90));

        Pose2d shootVolleyPose = new Pose2d(ballRow1.position.x,-23, Math.toRadians(-90));

        Pose2d ballRow2 = new Pose2d(23, -30, Math.toRadians(-90));
        Pose2d ballRow2End = new Pose2d(ballRow2.position.x, -44.2, Math.toRadians(-90));

        Pose2d ballRow3 = new Pose2d(35.5, -29, Math.toRadians(-90));

        // return to volley pose



        Actions.runBlocking(
                new SequentialAction(

                )
        );
    }
}
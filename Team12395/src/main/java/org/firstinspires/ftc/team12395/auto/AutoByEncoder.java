package org.firstinspires.ftc.team12395.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.team12395.RobotHardware;

import static org.firstinspires.ftc.team12395.RobotHardware.*;

@Autonomous(name =  "Auto By Encoder", group = "Robot")
public class AutoByEncoder extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init();
        robot.resetDriveEncoder();

        robot.limelight.start();

        waitForStart();

        robot.setShooterVelocity(800);
        robot.setHoodAngle(0);//0.65
        //5 deg

        //robot.setTurretPositionRelative(35);
        robot.driveStraight(AXIAL_SPEED, -40, 0);

        //robot.setTurretPositionAbsolute(90);
        //robot.processObelisk();
        //robot.pattern.
        // shooter setup

        telemetry.addData("Offset: ", robot.homeToAprilTag());
        telemetry.update();
        robot.setTurretPositionRelative(robot.homeToAprilTag());

        sleep(500);
        robot.setArmPos(0.7);
        sleep(750);
        robot.setArmPos(1);
        sleep(750);
        robot.spindexerHandler(120);

        robot.setHoodAngle(0.4);

        while(robot.spindexer.isBusy()){
            telemetry.addData("waiting for spindexer...", "");
            telemetry.update();
        }
        telemetry.clear();

        sleep(500);

        robot.setArmPos(0.7);
        sleep(750);
        robot.setArmPos(1);
        sleep(750);
        robot.spindexerHandler(120);

        while(robot.spindexer.isBusy()){
            telemetry.addData("waiting for spindexer...", "");
            telemetry.update();
        }
        telemetry.clear();

        sleep(500);

        robot.setArmPos(0.7);
        sleep(750);
        robot.setArmPos(1);
        sleep(750);

        //robot.setShooterVelocity(0);

        // finish shooting

        // start pickingg up

        robot.spindexerHandler(60);

        robot.turnToHeading(0.2, 45);

        robot.driveEncoder(AXIAL_SPEED, -10, 10, 10 ,-10);

        robot.setIntakeSpeed(-1000);

        robot.driveStraight(AXIAL_SPEED, 14, 45);
        robot.driveStraight(0.3, 4, 45);

        sleep(250);
        robot.spindexerHandler(120);
        sleep(600);
        robot.driveStraight(0.2, 5, robot.getHeading());
        sleep(650);
        robot.spindexerHandler(120);
        sleep(600);
        robot.driveStraight(0.2, 5, robot.getHeading());
        sleep(650);
        robot.setIntakeSpeed(0);

        robot.spindexerHandler(60);

        // end pick up

        // repeat shooting sequence

        robot.setTurretPositionRelative(45);
        robot.setShooterVelocity(800);
        robot.setHoodAngle(0.4);
        robot.driveStraight(AXIAL_SPEED, -22,robot.getHeading());


        robot.setTurretPositionRelative(robot.homeToAprilTag() - 5* (robot.homeToAprilTag()/Math.abs(robot.homeToAprilTag())) );
        sleep(500);

        robot.setArmPos(0.7);
        sleep(750);
        robot.setArmPos(1);
        sleep(750);
        robot.spindexerHandler(120);

        while(robot.spindexer.isBusy()){
            telemetry.addData("waiting for spindexer...", "");
            telemetry.update();
        }
        telemetry.clear();

        sleep(500);

        robot.setArmPos(0.7);
        sleep(750);
        robot.setArmPos(1);
        sleep(750);
        robot.spindexerHandler(120);

        while(robot.spindexer.isBusy()){
            telemetry.addData("waiting for spindexer...", "");
            telemetry.update();
        }
        telemetry.clear();

        sleep(500);

        robot.setArmPos(0.7);
        sleep(750);
        robot.setArmPos(1);
        sleep(750);

        robot.setShooterVelocity(0);

        // end shooting sequence

        // strafe to next balls
        /*

        robot.spindexerHandler(60);


        robot.driveEncoder(AXIAL_SPEED, -28, 28, 28, -28);

        robot.driveStraight(0.6, 24, 0);

        robot.setIntakeSpeed(-1000);

        sleep(650);
        robot.spindexerHandler(120);
        sleep(600);
        robot.driveStraight(0.2, 5, 0);
        sleep(650);
        robot.spindexerHandler(120);
        sleep(600);
        robot.driveStraight(0.2, 5, 0);
        sleep(650);
        robot.setIntakeSpeed(0);

         */

    }

}
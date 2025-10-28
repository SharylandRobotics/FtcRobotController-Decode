package org.firstinspires.ftc.team12395.auto;

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
        robot.init();
        robot.resetDriveEncoder();

        robot.limelight.start();

        waitForStart();

        robot.setShooterVelocity(800);
        robot.setHoodAngle(0.65);
        //5 deg

        robot.setTurretPositionRelative(35);
        robot.driveEncoder( AXIAL_SPEED, -36, 36, 36, -36);

        sleep(1200);

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

        while(robot.spindexer.isBusy()){
            telemetry.addData("waiting for spindexer...", "");
            telemetry.update();
        }
        telemetry.clear();

        sleep(750);

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

        sleep(750);

        robot.setArmPos(0.7);
        sleep(750);
        robot.setArmPos(1);
        sleep(750);

        //robot.setShooterVelocity(0);

        // finish shooting

        // start pickingg up

        robot.spindexerHandler(60);

        robot.driveEncoder(AXIAL_SPEED, -20, 20, 20 ,-20);

        robot.setIntakeSpeed(-1000);

        robot.driveStraight(AXIAL_SPEED, 10, 0);

        robot.driveStraight(0.2, 10, 0);
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

        robot.spindexerHandler(60);

        // end pick up

        // repeat shooting sequence

        robot.setShooterVelocity(850);
        robot.setHoodAngle(0.5);
        robot.driveStraight(AXIAL_SPEED, -30, 0);


        robot.setTurretPositionRelative(robot.homeToAprilTag());

        sleep(1000);

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

        sleep(750);

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

        sleep(750);

        robot.setArmPos(0.7);
        sleep(750);
        robot.setArmPos(1);
        sleep(750);

        robot.setShooterVelocity(0);

        // end shooting sequence

        // strafe to next balls

        robot.spindexerHandler(60);

        robot.driveEncoder(AXIAL_SPEED, 0, 48, 48, 0);

        robot.driveEncoder(AXIAL_SPEED, -24, 24, 24, -24);

        robot.setIntakeSpeed(-1000);

        robot.driveStraight(0.2, 10, 0);
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

    }

}
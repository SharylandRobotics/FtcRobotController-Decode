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
        robot.setHoodAngle(0.375);
        //5 deg

        robot.setTurretPositionRelative(35);
        robot.driveEncoder( AXIAL_SPEED, -36, 36, 36, -36);

        robot.setTurretPositionAbsolute(90);
        robot.processObelisk();
        //robot.pattern.
        // shooter setup

        sleep(1500);
        telemetry.addData("Offset: ", robot.homeToAprilTag());
        telemetry.update();
        robot.setTurretPositionRelative(robot.homeToAprilTag());

        sleep(1000);

        robot.setArmPos(0.7);
        sleep(1000);
        robot.setArmPos(1);
        sleep(1000);
        robot.setSpindexerRelativeAngle(120);

        while(robot.spindexer.isBusy()){
            telemetry.addData("waiting for spindexer...", "");
            telemetry.update();
        }
        telemetry.clear();

        sleep(1000);

        robot.setArmPos(0.7);
        sleep(1000);
        robot.setArmPos(1);
        sleep(1000);
        robot.setSpindexerRelativeAngle(120);

        while(robot.spindexer.isBusy()){
            telemetry.addData("waiting for spindexer...", "");
            telemetry.update();
        }
        telemetry.clear();

        sleep(1000);

        robot.setArmPos(0.7);
        sleep(1000);
        robot.setArmPos(1);
        sleep(1000);

        robot.setShooterVelocity(0);

        robot.setSpindexerRelativeAngle(60);


        sleep(1000);

        robot.driveEncoder(AXIAL_SPEED, -20, 20, 20 ,-20);



        sleep(1000);

        robot.setIntakeSpeed(-1000);

        robot.driveStraight(0.15, 10, 0);

        sleep(750);

        robot.setIntakeSpeed(0);







    }

}
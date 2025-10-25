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

        waitForStart();

        robot.setShooterVelocity(800);
        robot.setHoodAngle(0.375);
        robot.setTurretPositionAbsolute(5, 0.8);
        robot.setTurretPositionAbsolute(5, 0.8);
        robot.driveStraight(AXIAL_SPEED, -48 , 0);
        // shooter setup

        sleep(1500);

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
        robot.setTurretPositionAbsolute(40);
        robot.turnToHeading(YAW_SPEED, 44 );

        sleep(1000);

        robot.driveEncoder(AXIAL_SPEED, -13, 13, 13 ,-13);

        /*

        sleep(1000);

        robot.setIntakeSpeed(1000);

        robot.turnToHeading(YAW_SPEED, -45);
        robot.driveStraight(0.2, 10, -45);

         */



    }

}
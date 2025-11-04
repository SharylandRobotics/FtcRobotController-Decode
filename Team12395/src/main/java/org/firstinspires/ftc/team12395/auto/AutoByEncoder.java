package org.firstinspires.ftc.team12395.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.team12395.RobotHardware;

@Autonomous(name =  "Auto By Encoder", group = "Robot")
public class AutoByEncoder extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        robot.init();
        robot.resetDriveEncoder();

        waitForStart();

        // drive forward
        robot.driveEncoder(0.5, 39, 39, 39, 39);
        sleep(300);
        // strafe left
        robot.driveEncoder(0.5, -25, 25, 25, -25);
        sleep(300);
        // turn right
        robot.driveEncoder(0.5, 5, 5, -5, -5);
        sleep(300);

        robot.driveEncoder(1, -10, -10, -10, -10);
    }

}
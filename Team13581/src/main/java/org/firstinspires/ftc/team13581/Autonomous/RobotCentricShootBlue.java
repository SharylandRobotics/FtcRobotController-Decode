package org.firstinspires.ftc.team13581.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team13581.RobotHardware;

@Autonomous(name="Robot Centric Shoot Blue Team", group="Autonomous")

public class RobotCentricShootBlue extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode(){

        final double AXIAL_SPEED       = 0.4;
        final double LATERAL_SPEED     = 0.4;
        final double YAW_SPEED         = 0.2;
        robot.init();

        while(opModeInInit()) {
            // Display heading and status continuously during init loop
            telemetry.addData("Status", "Hardware Initialized");
            telemetry.addData("Heading", "%4.0f", robot.getHeading());
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        if (opModeIsActive()) {
            robot.setBackPower(0.75);
            robot.setAimPos(0.9);
            robot.setTurretPos(-5);
            robot.turnToHeading(YAW_SPEED, 50);
            robot.autoRobotCentric(AXIAL_SPEED, -48.0, 50.0);
            sleep(100);
            robot.setFrontPower(1);
            robot.setAimPos(0.8);
            sleep(200);
            robot.setFrontPower(0);
            sleep(1000);
            robot.setFrontPower(1);
            robot.setAimPos(0.7);
            sleep(1000);
            robot.setLeverPos(0.43);
            sleep(1000);
            robot.setLeverPos(0.17);
            robot.setFrontPower(0);
            robot.setBackPower(0);
            robot.setAimPos(0.2);
            sleep(1000);
            robot.turnToHeading(YAW_SPEED, 0.0);
            robot.holdHeading(YAW_SPEED, 0.0, 0.5);
            robot.autoRobotCentric(AXIAL_SPEED, -8.0, 0.0);
            robot.turnToHeading(YAW_SPEED,90.0);
            robot.holdHeading(YAW_SPEED, 90.0, 0.5);

            robot.setFrontPower(1);
            robot.setBackPower(-0.5);
            robot.autoRobotCentric(AXIAL_SPEED-0.25, 35.0, 90.0);
            robot.setFrontPower(0);
            robot.setBackPower(0);
            robot.autoRobotCentric(AXIAL_SPEED, -35.0, 90.0);


            robot.setBackPower(0.75);
            robot.setAimPos(0.9);
            robot.turnToHeading(YAW_SPEED, 50);
            robot.holdHeading(YAW_SPEED, 50, 0.5);
            robot.setFrontPower(-0.2);
            sleep(200);
            robot.setFrontPower(1);
            robot.setAimPos(0.8);
            sleep(200);
            robot.setFrontPower(0);
            sleep(1000);
            robot.setFrontPower(1);
            robot.setAimPos(0.7);
            sleep(1000);
            robot.setLeverPos(0.43);
            sleep(1000);
            robot.setLeverPos(0.17);
            robot.setFrontPower(0);
            robot.setBackPower(0);
            robot.setAimPos(0.2);
            sleep(2000);
            robot.turnToHeading(YAW_SPEED, 0.0);
            robot.holdHeading(YAW_SPEED, 0.0, 0.2);
            robot.autoRobotCentric(AXIAL_SPEED+0.2, -32.0, 0.0);

            sleep(1000);

            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);  // Pause to display last telemetry message.
        }
    }
}
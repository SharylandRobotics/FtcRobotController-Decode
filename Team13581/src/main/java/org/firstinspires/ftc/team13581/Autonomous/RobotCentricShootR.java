package org.firstinspires.ftc.team13581.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team13581.RobotHardware;

@Autonomous(name="Robot Centric Shoot Blue Team", group="Autonomous")

public class RobotCentricShootR extends LinearOpMode {

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
            robot.autoRobotCentric(AXIAL_SPEED, -52.0, 0.0);
            robot.setBackPower(0.8);
            robot.setAimPos(0.45);
            sleep(2000);
            robot.setFrontPower(1);
            sleep(200);
            robot.setFrontPower(0);
            sleep(1000);
            robot.setFrontPower(1);
            sleep(1000);
            robot.setLeverPos(0.43);
            sleep(2000);
            robot.setLeverPos(0.17);
            robot.setFrontPower(0);
            robot.setBackPower(0);
            robot.setAimPos(0.2);
            sleep(2000);
            robot.turnToHeading(YAW_SPEED, -45.0);
            robot.holdHeading(YAW_SPEED, -45.0, 0.5);

            robot.autoRobotCentric(AXIAL_SPEED, 17.0, -45.0);
            sleep(1000);

            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);  // Pause to display last telemetry message.
        }
    }
}
package org.firstinspires.ftc.team13590;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team13590.RobotHardware;

import static org.firstinspires.ftc.team13590.RobotHardware.*;

@Autonomous(name = "Gyro", group = "opMode")

// Autonomous routine using gyro-based driving with RobotHardware helpers
public class Auto extends LinearOpMode {

    // Instantiate RobotHardware and link this OpMode
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {

        // Initialize all motors and IMU before start
        robot.init();

        while(opModeInInit()) {
            // Display heading and status continuously during init loop
            telemetry.addData("Status", "Hardware Initialized");
            telemetry.addData("Heading", "%4.0f", robot.getHeading());
            telemetry.update();
        }

        // Wait for PLAY; exit early if stop is pressed
        waitForStart();
        if (isStopRequested()) return;

        // Execute full autonomous path sequence once started
        if (opModeIsActive()) {

            // Drive 24" forward, then turn and hold headings as defined
            robot.driveStraight(MAX_AUTO_AXIAL, 24.0, 0.0);
//            robot.turnToHeading(MAX_AUTO_YAW, -45.0);
//            robot.holdHeading(MAX_AUTO_YAW, -45.0, 0.5);
//
//            robot.driveStraight(MAX_AUTO_AXIAL, 17.0, -45.0);
//            robot.turnToHeading(MAX_AUTO_YAW, 45.0);
//            robot.holdHeading(MAX_AUTO_YAW, 45.0, 0.5);
//
//            robot.driveStraight(MAX_AUTO_AXIAL, 17.0, 45.0);
//            robot.turnToHeading(MAX_AUTO_YAW, 0.0);
//            robot.holdHeading(MAX_AUTO_YAW, 0.0, 1.0);
//
//            robot.driveStraight(MAX_AUTO_AXIAL, -48.0, 0.0);

            // Indicate completion and pause for display
            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);  // Pause to display last telemetry message.
        }
    }
}

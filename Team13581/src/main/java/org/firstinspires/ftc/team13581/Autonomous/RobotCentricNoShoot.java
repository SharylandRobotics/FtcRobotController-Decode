package org.firstinspires.ftc.team13581.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team13581.RobotHardware;

@Autonomous(name="Robot Centric No Shoot", group="Autonomous")

public class RobotCentricNoShoot extends LinearOpMode {

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

        // Wait for PLAY; exit early if stop is pressed
        waitForStart();
        if (isStopRequested()) return;

        // Execute full autonomous path sequence once started
        if (opModeIsActive()) {
            // Drive 24" forward, then turn and hold headings as defined
            robot.autoRobotCentric(AXIAL_SPEED, 48.0, 0.0);

            //robot.autoRobotCentric(AXIAL_SPEED, 24.0, 0.0);
            //robot.turnToHeading(YAW_SPEED, 0);
            //robot.holdHeading(YAW_SPEED, 0, 0.5);

            // Indicate completion and pause for display
            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);  // Pause to display last telemetry message.
        }
    }
}

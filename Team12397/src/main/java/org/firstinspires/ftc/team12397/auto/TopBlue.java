package org.firstinspires.ftc.team12397.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team12397.RobotHardware;


@Autonomous(name = "TopBlue", group = "opMode")
public class TopBlue extends LinearOpMode {


    // Instantiate RobotHardware
    RobotHardware robot = new RobotHardware(this);


    @Override
    public void runOpMode() {


        // Target turret velocities
        double FIRST_VELOCITY = 1300;
        double SECOND_VELOCITY = 1300;


        // Initialize all motors and IMU before start
        robot.init();


        // Display heading during init loop
        while (opModeInInit()) {
            telemetry.addData("Status", "Hardware Initialized");
            telemetry.addData("Heading", "%4.0f", robot.getHeading());
            telemetry.update();
        }


        // Wait for PLAY; exit early if stop is pressed
        waitForStart();
        if (isStopRequested()) return;


        // === Autonomous routine ===
        robot.turretVelocity(FIRST_VELOCITY);


        robot.driveStraight(.7, -50, 0);
        robot.holdHeading(robot.YAW_SPEED, 0, 1);
        robot.turnToHeading(1, -7);
        robot.holdHeading(robot.YAW_SPEED, -7, 1);


        // Wait until turret reaches speed while holding heading
        while (robot.getVelocity() < FIRST_VELOCITY-20) {
            robot.holdHeading(1, -7, 0.1);
        }


        robot.intakePower(-1);
        robot.holdHeading(0, -7, 3);


        // Turn off motors
        robot.turretVelocity(0);
        robot.intakePower(0);


        // Phase 2 movement
        robot.turnToHeading(0.5, 40);
        robot.holdHeading(0.5, 40, 1);
        robot.straif(0.5, -7, 40);
        robot.intakePower(-1);
        robot.driveStraight(0.3, 18, 40);
        robot.driveStraight(0.3, 15, 40);
        robot.straif(0.5, 1, 40);


        // Extra heading hold and turret prep
        robot.holdHeading(robot.YAW_SPEED, 40, 1);
        //robot.intakePower(0.1); // unblock motor
        //robot.holdHeading(robot.YAW_SPEED, -40, 1);
        robot.intakePower(0);
        robot.turretVelocity(SECOND_VELOCITY);


        // Shooting movement
        robot.driveStraight(.7, -33, 40);
        robot.holdHeading(robot.YAW_SPEED, 40, 1);
        robot.turnToHeading(1, -7);
        robot.holdHeading(robot.YAW_SPEED, -7, 1);


        // Wait until turret reaches shooting speed
        while (robot.getVelocity() < SECOND_VELOCITY-20) {
            robot.holdHeading(1, -7, 0.1);
        }


        robot.intakePower(-1);
        robot.holdHeading(0, -7, 3);


        // Move out
        robot.straif(1, -22, 0);
    }
}

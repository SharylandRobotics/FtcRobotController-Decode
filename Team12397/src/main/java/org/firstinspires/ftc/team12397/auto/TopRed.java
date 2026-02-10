package org.firstinspires.ftc.team12397.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team12397.RobotHardware;


@Autonomous(name = "TopRed", group = "opMode")
public class TopRed extends LinearOpMode {


    // Instantiate RobotHardware
    RobotHardware robot = new RobotHardware(this);


    @Override
    public void runOpMode() {


        // Target turret velocities
        double FIRST_VELOCITY = 1000;
        double SECOND_VELOCITY = 1000;


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
        robot.setHoodPositions(1);
        robot.turretVelocity(FIRST_VELOCITY);


        robot.driveStraight(1, -50, 0);
        robot.holdHeading(robot.YAW_SPEED, 0, 1);
        robot.turnToHeading(1, 5);


        // Wait until turret reaches speed while holding heading
        while (opModeIsActive() && robot.getVelocity() < FIRST_VELOCITY-20) {
            robot.holdHeading(1, 5, 0.1);
        }


        robot.intakePower(-1);
        robot.holdHeading(0, 5, 3);


        // Turn off motors
        robot.turretVelocity(0);
        robot.intakePower(0);


        // Phase 2 movement
        robot.turnToHeading(0.5, -40);
        robot.holdHeading(0.5, -40, 1);
        robot.straif(0.5, 8, -40);
        robot.intakePower(-1);
        robot.driveStraight(0.5, 18, -40);
        robot.driveStraight(0.5, 13, -40);


        // Extra heading hold and turret prep
        robot.holdHeading(robot.YAW_SPEED, -40, 1);
        //robot.intakePower(0.1); // unblock motor
        //robot.holdHeading(robot.YAW_SPEED, -40, 1);
        robot.intakePower(0);
        robot.turretVelocity(SECOND_VELOCITY);


        // Shooting movement
        robot.driveStraight(1, -31, -40);
        robot.holdHeading(robot.YAW_SPEED, -40, 1);
        robot.turnToHeading(1, 3);
        robot.holdHeading(robot.YAW_SPEED, 3, 1);


        // Wait until turret reaches shooting speed
        while (opModeIsActive() && robot.getVelocity() < SECOND_VELOCITY-20) {
            robot.holdHeading(1, 3, 0.1);
        }


        robot.intakePower(-1);
        robot.holdHeading(0, 3, 3);


        // Move out
        robot.straif(1, 20, 0);
    }
}



package org.firstinspires.ftc.team12397;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


// Drivetrain motors for a mecanum chassis

@TeleOp(name="Test Code", group="TeleOp")
// TODO(STUDENTS): You may rename this OpMode in the annotation for clarity (e.g., "Robot-Centric - Practice Bot")

public class AntMotorCode extends LinearOpMode {
    private RobotHardware robot = new RobotHardware(this);

    public void runOpMode() {
        // Driver inputs (range about [-1, 1])
        double axial = 0; // forward/back (+ forward)
        double lateral = 0; // strafe left/right (+ right)
        double yaw = 0; // rotation (+ CCW/left turn)
        double currentVelocity=0;
        double maxVelocity=0;
        Boolean on = false;
        // --- INIT PHASE ---
        // WHY: Centralized initialization (motor directions, encoders, IMU) lives in RobotHardware.init()
        // TODO(STUDENTS): Confirm motor names, directions, and zero-power modes in RobotHardware.init()
        robot.init();

        // Wait for START on the Diver Station
        waitForStart();

        // --- TELEOP LOOP ---
        while (opModeIsActive()) {

            // Read sticks (FTC gamepads: pushing left_stick_y forward is negative → invert it)
            axial = -gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;
            robot.driveFieldCentric(axial, lateral, yaw);
            if (gamepad1.right_trigger >0.5){
                on = !on;
            }
            if (on){
                robot.turretVelocity(100);

                currentVelocity= robot.getVelocity();
                if (currentVelocity > maxVelocity) {
                    maxVelocity = currentVelocity;
                }


            }
            else{
                robot.turretPower(0);
            }
            telemetry.addData("current velocity", currentVelocity);
            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.update();



        }
    }
}
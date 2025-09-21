package org.firstinspires.ftc.team00000.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team00000.RobotHardware;

/**
 * Robot-centric TeleOp
 * – Useful for driver training and quick drive diagnostics (IMU not required).
 * – Same driver aids as FieldCentric for consistency
 */
@TeleOp(name="Robot Centric", group="TeleOp")
public class RobotCentric extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        double axial = 0, lateral = 0, yaw = 0;

        robot.init();

        telemetry.addLine("Robot Centric: RT=slow mode, Start=zero heading");
        telemetry.update();

        waitForStart();
        robot.start();

        while (opModeIsActive() && !isStopRequested()) {
            axial   = -gamepad1.left_stick_y;
            lateral =  gamepad1.left_stick_x;
            yaw     =  gamepad1.right_stick_x;

            double scale = gamepad1.right_trigger > 0.1 ? 0.35 : 1.0;
            axial *= scale; lateral *= scale; yaw *= scale;

            if (gamepad1.start) robot.start(); // keep the UX consistent; harmless in robot-centric

            robot.teleOpRobotCentric(axial, lateral, yaw);
            robot.periodic();

            // Vision + shooter telemetry using RobotHardware helpers
            robot.telemetryVision(telemetry);
            double suggested = robot.getSuggestedShooterRpm();
            if (suggested > 0) telemetry.addData("Shooter", "suggested=%.0f rpm", suggested);
            else               telemetry.addLine("Shooter: no suggestion");

            telemetry.addData("Heading(deg)", "%.1f", robot.getHeading());
            telemetry.addData("Inputs", "ax=%.2f lat=%.2f yaw=%.2f slow=%.2f",
                    axial, lateral, yaw, scale);
            // Optional: press A to auto-nudge to ~2.50 m from the tag while holding current heading
            if (gamepad1.a) {
                robot.assistDriveToTagRange(2.50, 0.05, 0.25);
            }
            telemetry.update();

            idle();
        }

        robot.stop();
    }
}
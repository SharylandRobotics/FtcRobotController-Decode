package org.firstinspires.ftc.team00000.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team00000.RobotHardware;

/**
 * Field-centric TeleOp
 * – Left stick: axial/lateral in field frame; Right stick: yaw.
 * – Includes lifestyle calls and cooperative yielding (idle).
 * – Adds heading telemetry for quick sanity checks/
 */
@TeleOp(name="Field Centric", group="TeleOp")
public class FieldCentric extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware(this);
    private boolean prevY = false;
    private boolean prevStart = false;

    @Override
    public void runOpMode() {
        double axial = 0, lateral = 0, yaw = 0;

        robot.init();
        telemetry.setMsTransmissionInterval(50); // faster telemetry updates

        telemetry.addLine("Field Centric: RT=slow mode, Start=zero heading");
        int heartbeat = 0;
        telemetry.addData("Heartbeat", heartbeat);
        telemetry.update();

        waitForStart();
        robot.start();

        while (opModeIsActive() && !isStopRequested()) {
            // Sticks (FTC convention: forward stick is negative)
            axial   = -gamepad1.left_stick_y;
            lateral =  gamepad1.left_stick_x;
            yaw     =  gamepad1.right_stick_x;

            // Optional slow mode for precise work
            double scale = gamepad1.right_trigger > 0.1 ? 0.35 : 1.0;
            axial *= scale; lateral *= scale; yaw *= scale;

            // Re-zero heading on START (edge triggered)
            if (gamepad1.start && !prevStart) {
                robot.start();
            }
            prevStart = gamepad1.start;

            // Toggle vision heading blend with Y (edge triggered)
            if (gamepad1.y && !prevY) {
                RobotHardware.VISION_HEADING_BLEND_ENABLED = !RobotHardware.VISION_HEADING_BLEND_ENABLED;
            }
            prevY = gamepad1.y;

            robot.teleOpFieldCentric(axial, lateral, yaw);
            robot.periodic();

            telemetry.addData("Heading(deg)", "%.1f", robot.getHeading());
            heartbeat++;
            telemetry.addData("Heartbeat", heartbeat);

            if (gamepad1.a) {
                robot.assistDriveToTagRange(2.50, 0.05, 0.25);
            }

            telemetry.update();

            idle(); // cooperative yield (preferred to fixed sleep)
        }

        robot.stop();
    }
}
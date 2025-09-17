package org.firstinspires.ftc.team00000.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team00000.RobotHardware;

/**
 * Field-centric TeleOp
 * – Left stick: axial/lateral in field frame: Right sick: yaw.
 * – Includes lifestyle calls and cooperative yielding (idle).
 * – Adds heading telemetry for quick sanity checks/
 */
@TeleOp(name="Field Centric", group="TeleOp")
public class FieldCentric extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        double axial = 0, lateral = 0, yaw = 0;

        robot.init();

        telemetry.addLine("Field Centric: RT=slow mode, Start=zero heading");
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

            // Re-zero heading if the hub was bumped
            if (gamepad1.start) robot.start();

            robot.teleOpFieldCentric(axial, lateral, yaw);
            robot.periodic();

            telemetry.addData("Heading(deg)", "%.1f", robot.getHeading());
            telemetry.addData("Inputs", "ax=%.2f lat=%.2f yaw=%.2f slow=%.2f",
                    axial, lateral, yaw, scale);
            telemetry.update();

            idle(); // cooperative yield (preferred to fixed sleep)
        }

        robot.stop();
    }
}
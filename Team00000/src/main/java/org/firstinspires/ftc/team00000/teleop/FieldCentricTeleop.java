package org.firstinspires.ftc.team00000.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team00000.RobotHardware;

@TeleOp(name = "Field Centric", group = "TeleOp")
public class FieldCentricTeleop extends LinearOpMode {

    private RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        robot.init();

        telemetry.addLine("Ready. (RT = slow mode, Y = zero heading)");
        telemetry.update();

        waitForStart();
        robot.start();

        while (opModeIsActive() && !isStopRequested()) {

            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            double slowScale = gamepad1.right_trigger > 0.1 ? 0.35 : 1.0;
            axial   *= slowScale;
            lateral *= slowScale;
            yaw     *= slowScale;

            // Zero heading with Y (for future auto/field use)
            if (gamepad1.y) robot.start();

            robot.teleOpFieldCentric(axial, lateral, yaw);
            robot.periodic();

            telemetry.addData("Heading (deg)", "%.1f", robot.getHeading());
            telemetry.addData("Inputs", "ax=%.2f lat=%.2f yaw=%.2f slow=%.2f",
                    axial, lateral, yaw, slowScale);
            telemetry.update();

            idle();
        }

        robot.stop();
    }
}
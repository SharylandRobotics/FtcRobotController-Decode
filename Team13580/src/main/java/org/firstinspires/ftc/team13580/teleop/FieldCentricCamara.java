package org.firstinspires.ftc.team13580.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team13580.RobotHardware;

@TeleOp(name = "Field Centric Camara", group = "opMode")
public class FieldCentricCamara extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        double kickerBackPos = -1;
        double kickerForwardPos = 1;
        double kickerLeftBackPos = -1;
        double kickerLeftForwardPos = 1;

        double axial, lateral, yaw;
        double intake, outtake = 0;  // Initialize outtake here

        // Toggle variables
        boolean outtakeManual = false;

        robot.init();

        while (opModeInInit()) {
            telemetry.addData("Status", "Hardware Initialized");
            telemetry.addData("Vision", "Ready (AprilTag)");
            telemetry.addData("Mode", "INIT");
            telemetry.addData("Heading", "%4.0f", robot.getHeading());
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Update camera
            robot.updateAprilTagDetections();

            // Precision mode
            boolean slow = gamepad1.left_bumper;
            double scale = slow ? 0.4 : 1.0;

            // Joystick input
            axial = -gamepad1.left_stick_y * scale;
            lateral = gamepad1.left_stick_x * scale;
            yaw = gamepad1.right_stick_x * scale;

            // Auto-drive using camera
            boolean didAuto = false;

            // Drive robot if not auto-driving
            if (!didAuto) {
                robot.driveFieldCentric(axial, lateral, yaw);
            }

            // Intake control
            if (gamepad1.right_trigger == 1) {
                intake = 0.6;
            } else if (gamepad1.dpad_down) {
                intake = -1;
            } else if (gamepad1.right_bumper) {
                intake = 0.3;
            } else {
                intake = 0;
            }
            robot.setIntakePower(intake);

            // Outtake toggle with X button
            if (gamepad1.xWasPressed()) {
                outtakeManual = !outtakeManual;  // toggle on button press
            }

            // Set outtake speed
            if (outtakeManual) {
                if (!Double.isNaN(robot.getGoalRangeIn())) {
                    outtake = robot.getCalculatedVelocity(robot.getFloorDistance());  // manual toggle speed
                }
            } else {
                outtake = gamepad1.left_trigger * 2000;
            }
            robot.setOuttakeVelocity((int) outtake);

            // Kicker control
            if (gamepad1.a) {
                robot.setKickerPower(kickerBackPos);
            } else {
                robot.setKickerPower(kickerForwardPos);
            }

            if (gamepad1.b) {
                robot.setKickerLeftPower(kickerLeftBackPos);
            } else {
                robot.setKickerLeftPower(kickerLeftForwardPos);
            }

            if (gamepad2.a) {
                robot.setKickerPower(kickerBackPos);
            } else {
                robot.setKickerPower(kickerForwardPos);
            }

            if (gamepad2.b) {
                robot.setKickerLeftPower(kickerLeftBackPos);
            } else {
                robot.setKickerLeftPower(kickerLeftForwardPos);
            }

            if(gamepad1.y){
                robot.setKickerLeftPower(kickerLeftBackPos);
                robot.setKickerPower(kickerBackPos);
            } else {
                robot.setKickerLeftPower(kickerLeftForwardPos);
                robot.setKickerPower(kickerForwardPos);
            }

            if (gamepad2.yWasPressed()){
                robot.setOuttakeVelocity(1700);
            }

            // Telemetry
            telemetry.addData("Mode", slow ? "SLOW" : "NORMAL");
            telemetry.addData("Outtake Toggle", outtakeManual ? "ON (1300)" : "OFF");
            telemetry.addData("Heading", "%4.0f°", robot.getHeading());
            telemetry.addData("Drive", "ax=%.2f  lat=%.2f  yaw=%.2f", axial, lateral, yaw);
            telemetry.update();

            sleep(50);
        }
    }
}

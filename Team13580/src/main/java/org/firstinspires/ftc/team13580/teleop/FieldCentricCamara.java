package org.firstinspires.ftc.team13580.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team13580.RobotHardware;

@TeleOp(name = "Field Centric Camara Toggle Outtake", group = "opMode")
public class FieldCentricCamara extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        double kickerBackPos = 0;
        double kickerForwardPos = 1;
        double kickerLeftBackPos = 0;
        double kickerLeftForwardPos = 1;

        double axial, lateral, yaw;
        double intake, outtake = 0;  // Initialize outtake here

        // Toggle variables
        boolean outtakeManual = false;
        boolean xPreviouslyPressed = false;

        boolean autoSetSped = false;

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

            robot.driveFieldCentric(axial, lateral, yaw);

            if (gamepad1.xWasPressed()){
                autoSetSped = !autoSetSped;
            }

            if (autoSetSped && !Double.isNaN(robot.getGoalRangeIn())){
                outtake = robot.getCalculatedVelocity(robot.getFloorDistance());
            }


            // Intake control
            if (gamepad1.right_trigger == 1) {
                intake = 0.5;
            } else if (gamepad1.dpad_down) {
                intake = -1;
            } else if (gamepad1.right_bumper) {
                intake = 0.3;
            } else {
                intake = 0;
            }
            robot.setIntakePower(intake);

            // Outtake toggle with X button
            if (gamepad1.x && !xPreviouslyPressed) {
                outtakeManual = !outtakeManual;  // toggle on button press
            }
            xPreviouslyPressed = gamepad1.x;

           if (gamepad1.y) {
                outtake = 1300;  // manual toggle speed
           } else if (!autoSetSped){
                outtake = gamepad1.left_trigger * 2000;
           }
           robot.setOuttakeVelocity((int) outtake);

            // Kicker control
            robot.setKickerPower(gamepad1.b ? kickerForwardPos : kickerBackPos);
            robot.setKickerLeftPower(gamepad1.a ? kickerLeftForwardPos : kickerLeftBackPos);

            // Telemetry
            telemetry.addData("Mode", slow ? "SLOW" : "NORMAL");
            telemetry.addData("Vel : ", outtake);
            telemetry.addData("Heading", "%4.0f°", robot.getHeading());
            telemetry.addData("Drive", "ax=%.2f  lat=%.2f  yaw=%.2f", axial, lateral, yaw);
            telemetry.addData("Distnace: ", robot.getFloorDistance());
            telemetry.addData("nDistance: ", robot.getGoalRangeIn());
            telemetry.update();

            sleep(50);
        }
    }
}

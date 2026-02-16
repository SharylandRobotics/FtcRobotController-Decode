package org.firstinspires.ftc.team13580.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team13580.RobotHardware;

@TeleOp(name = "Field Centric Solo", group = "opMode")
public class FieldCentricSolo extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        double kickerBackPos = -1;
        double kickerForwardPos = .3;
        double kickerLeftBackPos = -1;
        double kickerLeftForwardPos = .3;

        double axial, lateral, yaw;
        double intake, outtake = 0;  // Initialize outtake here

        // Toggle variables
        boolean outtakeManual = false;
        double yawCorrection = 0;

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


            boolean bearingAssist = gamepad1.right_bumper;

            if (bearingAssist) {
                double correction = robot.autoBearingToGoalCorrect();
                telemetry.addData("Correcting by: ", correction);
                if (!Double.isNaN(correction)){
                    yawCorrection = correction;
                    robot.driveFieldCentric(axial, lateral, yawCorrection);
                    telemetry.addData("CORRECTING","");
                } else {
                    telemetry.addData("INVALID","");
                    robot.driveFieldCentric(axial, lateral, yaw);
                }
            } else {
                robot.driveFieldCentric(axial, lateral, yaw);
            }

            // Drive robot if not auto-driving



            // Intake control
            if (gamepad1.right_trigger == 1) {
                intake = 1;
            } else if (gamepad1.dpad_down) {
                intake = -1;
            } else if (gamepad1.right_bumper) {
                intake = 0.3;
            } else {
                intake = .6;
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

            if(gamepad1.y){
                robot.setKickerLeftPower(kickerLeftBackPos);
                robot.setKickerPower(kickerBackPos);
            } else {
                robot.setKickerLeftPower(kickerLeftForwardPos);
                robot.setKickerPower(kickerForwardPos);
            }



            if (gamepad2.aWasPressed()){
                robot.setOuttakeVelocity(1700);
            }

            // Telemetry
            telemetry.addData("Mode", slow ? "SLOW" : "NORMAL");
            telemetry.addData("Outtake Toggle", outtakeManual ? "ON (1300)" : "OFF");
            telemetry.addData("Heading", "%4.0f°", robot.getHeading());
            telemetry.addData("Drive", "ax=%.2f  lat=%.2f  yaw=%.2f", axial, lateral, yaw);
            telemetry.addData("Camera Values: ", "dist=%.2f  bearing=%.2f  yaw=%.2f", robot.getFloorDistance(), robot.getGoalBearingDeg(), robot.getTagYawDeg());
            telemetry.update();

            sleep(50);
        }
    }
}

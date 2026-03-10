package org.firstinspires.ftc.team13580.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team13580.RobotHardware;

@TeleOp(name = "Field Centric Camara", group = "opMode")
@Config
public class FieldCentricCamara extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    public static double kP = 0.03;
    public static double kI = 0;
    public static double kD = 0.0001;
    public static double kF = 1;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        double kickerBackPos = -1;
        double kickerForwardPos = .2;
        double kickerLeftBackPos = -1;
        double kickerLeftForwardPos = .2;

        double axial, lateral, yaw;
        double intake, outtake = 0;  // Initialize outtake here

        // Toggle variables
        boolean outtakeManual = false;
        boolean outtakeFarToggle = false;

        boolean leftKickerTimer = false;
        boolean rightKickerTimer = false;

        double leftKickerClock = 0;
        double rightKickerClock = 0;

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
            //  boolean slow = gamepad1.left_bumper;
            // double scale = slow ? 0.4 : 1.0;

            // Joystick input
            axial = -gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;

            // Auto-drive using camera
            boolean autoTurn = gamepad1.left_bumper;

            robot.pidfController.setP(kP);
            robot.pidfController.setI(kI);
            robot.pidfController.setD(kD);
            robot.pidfController.setF(kF);

            // Drive robot if not auto-driving
            if (autoTurn && !Double.isNaN(robot.getGoalBearingDeg())) {
                yaw = robot.autoBearingToGoalCorrect();
            }

            robot.driveFieldCentric(axial, lateral, yaw);

            // Intake control
            if (gamepad1.right_trigger == 1) {
                intake = 1;
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
                // when turned off, set velocity to 0
                if (!outtakeManual) {
                    outtake = 0;
                }
            }

             if (gamepad1.dpad_down){
                 outtakeFarToggle = !outtakeFarToggle;
               }

            // Set outtake speed
                if (outtakeManual) {
                    if (!Double.isNaN(robot.getGoalRangeIn())) {
                         outtake = robot.getCalculatedVelocity(robot.getFloorDistance());  // manual toggle speed
                    }
                }
                 if (outtakeFarToggle) {
                    outtake = 1500;
               }
                 robot.setOuttakeVelocity((int) outtake);

            // Kicker control

            if (gamepad1.a || rightKickerTimer) {
                if (!rightKickerTimer){
                    leftKickerTimer = true;
                    leftKickerClock = 15;
                }
                robot.setKickerPower(kickerBackPos);
            } else if (gamepad1.y) {
                robot.setKickerPower(kickerBackPos);
            } else {
                robot.setKickerPower(kickerForwardPos);
            }

            if (gamepad1.b || leftKickerTimer) {
                if (!leftKickerTimer){
                    rightKickerTimer = true;
                    rightKickerClock = 15;
                }
                robot.setKickerLeftPower(kickerLeftBackPos);
            } else if (gamepad1.y) {
                robot.setKickerLeftPower(kickerLeftBackPos);
            } else {
                robot.setKickerLeftPower(kickerLeftForwardPos);
            }

            //if(gamepad2.y){
               // robot.setKickerLeftPower(kickerLeftBackPos);
             //   robot.setKickerPower(kickerBackPos);
          //  } else if (gamepad1.y) {
               // robot.setKickerLeftPower(kickerLeftBackPos);
              //  robot.setKickerPower(kickerBackPos);
          //  } else {
              //  robot.setKickerLeftPower(kickerLeftForwardPos);
              //  robot.setKickerPower(kickerForwardPos);
           // }

            // Telemetry
           // telemetry.addData("Mode", slow ? "SLOW" : "NORMAL");
            telemetry.addData("Outtake Toggle", outtakeManual ? "ON (1300)" : "OFF");
            telemetry.addData("Heading", "%4.0f°", robot.getHeading());
            telemetry.addData("Drive", "ax=%.2f  lat=%.2f  yaw=%.2f", axial, lateral, yaw);
            telemetry.update();

            sleep(20);

            if (rightKickerClock < 0){
                rightKickerTimer = false;
                rightKickerClock = 0;
            } else {
                rightKickerClock--;
            }

            if (leftKickerClock < 0){
                leftKickerTimer = false;
                leftKickerClock = 0;
            } else {
                leftKickerClock--;
            }
        }
    }
}

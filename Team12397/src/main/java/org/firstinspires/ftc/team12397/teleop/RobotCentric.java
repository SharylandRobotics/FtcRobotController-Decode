package org.firstinspires.ftc.team12397.teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.team12397.RobotHardware;


@TeleOp(name = "Robot Centric", group = "TeleOp")
public class RobotCentric extends LinearOpMode {


    RobotHardware robot = new RobotHardware(this);


    enum TurretState {
        Start, HoldWait, Fire, End
    }


    TurretState turretState = TurretState.Start;
    ElapsedTime turretTimer = new ElapsedTime();
    String statusMsg = "";
    public boolean isTimeUp(double seconds) {
        if (turretTimer.milliseconds() >= seconds * 1000) {
            turretTimer.reset(); // Auto-reset for the next state
            return true;
        }
        return false;
    }


    @Override
    public void runOpMode() {





        double axial = 0, lateral = 0, yaw = 0;
        boolean servoOn = false;
        boolean shooterOn = false;
        boolean intakeOn = false;


        double VELOCITY = 1500;


        // --- INIT PHASE ---
        robot.init();
        while (opModeInInit()) {
            telemetry.addData("Status", "Hardware Initialized");
            telemetry.addData("Heading", "%4.0f", robot.getHeading());
            telemetry.addData("Velocity", robot.getVelocity());
            telemetry.addData("Vision", "Ready (AprilTag)");
            telemetry.update();
        }


        waitForStart();
        if (isStopRequested()) return;


        while (opModeIsActive()) {


            // --- AUTO SHOOTER STATE MACHINE ---
            switch (turretState) {
                case Start:
                    if (gamepad1.right_trigger > 0.5) {
                        statusMsg = "RUNNING AUTO SHOOTER";
                        robot.turretVelocity(VELOCITY);
                        turretState = TurretState.HoldWait;
                    }
                    break;


                case HoldWait:
                    if (robot.getVelocity() >= VELOCITY-20) {
                        robot.intakePower(-1);
                        turretTimer.reset();
                        turretState = TurretState.Fire;
                    }
                    break;


                case Fire:
                    // Fire until manually stopped or velocity drops
                    if (isTimeUp(4.0)) {
                        turretState = TurretState.End;
                    }
                    break;


                case End:
                    statusMsg = "";
                    robot.turretVelocity(0);
                    robot.intakePower(0);
                    turretState = TurretState.Start;
                    break;
            }


            // --- KILL SWITCH ---
            if (gamepad1.y && turretState != TurretState.Start) {
                robot.setIntakeServo(1);
                robot.turretVelocity(0);
                robot.intakePower(0);
                turretState = TurretState.Start;
                statusMsg = "KILLED";
                turretTimer.reset();
            }


            // --- MANUAL CONTROLS WHEN NOT AUTO SHOOTING ---
            if (turretState == TurretState.Start) {


                // Manual intake
                if (gamepad1.left_trigger > 0.5) robot.intakePower(-1);
                else if (gamepad1.x) robot.intakePower(1);
                else robot.intakePower(0);


                // Manual intake servo
                robot.setIntakeServo(gamepad1.a ? 0 : 1);


                // Hood toggle
                if (gamepad1.b) servoOn = !servoOn;
                robot.setHoodPositions(servoOn ? 1 : .8);


                // Manual turret
                robot.turretVelocity(gamepad1.left_bumper ? VELOCITY : 0);
            }


            // --- DRIVE INPUTS ---
            boolean slow = gamepad1.left_bumper;
            double scale = slow ? 1.0 : 1.0;
            axial = -gamepad1.left_stick_y * scale;
            lateral = gamepad1.left_stick_x * scale;
            yaw = gamepad1.right_stick_x * scale;


            // Auto-assist toward visible goal
            boolean autoAssist = gamepad1.right_bumper;
            boolean didAuto = false;
            if (autoAssist) {
                robot.updateAprilTagDetections();
                didAuto = robot.autoDriveToGoalStep();
            }


            // Field-centric drive if auto not applied
            if (!didAuto) robot.driveRobotCentric(axial, lateral, yaw);


            // --- TELEMETRY ---
            telemetry.addData("Turret Speed", robot.getVelocity());
            if (statusMsg.equals("KILLED") && turretTimer.milliseconds() < 3000) {
                telemetry.addLine("-----------------------------");
                telemetry.addLine("      SYSTEM KILLED          ");
                telemetry.addLine("-----------------------------");
            }
            if (statusMsg.equals("RUNNING AUTO SHOOTER")) {
                telemetry.addLine("-----------------------------");
                telemetry.addLine(statusMsg);
                telemetry.addLine("-----------------------------");
            }
            telemetry.addLine("Gamepad2 = driver. Gamepad1 = turret controls");
            telemetry.addLine("Right Trigger Toggle = Auto Turret");
            telemetry.addLine("Y = Auto shooter kill switch");
            telemetry.addData("Turn to tag", "Hold Right Bumper");
            telemetry.addLine("Left Bumper Hold = Manual Turret (Auto Off)");
            telemetry.addLine("Left Trigger Hold = Intake");
            telemetry.addLine("B = Hood Servo");
            telemetry.addLine("A = Intake Servo");
            telemetry.addLine("X = Reverse Intake");
            telemetry.update();


            sleep(50);
        }
    }
}

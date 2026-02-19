package org.firstinspires.ftc.team12397;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TwoPlayerRC", group = "TeleOp")
// TODO(STUDENTS): You may rename this OpMode in the annotation for clarity (e.g., "Robot-Centric - Practice Bot")
public class TwoPlayerRC extends LinearOpMode {


    // NOTE: One RobotHardware instance per OpMode keeps mapping/telemetry simple.
    RobotHardware robot = new RobotHardware(this);

    enum TurretState {
        Start, HoldWait, Fire, ResetIntake, End

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
        // Driver inputs (range roughly [-1, 1])
        double axial = 0; // forward/back (+ forward)
        double lateral = 0; // strafe left/right (+ right)
        double yaw = 0; // rotation (+ CCW/left turn)

        boolean servoOn = false;
        boolean lastServoState = false;

        boolean shooterOn = false;
        boolean lastShooterState = false;

        boolean intakeOn = false;
        boolean lastIntakeState = false;


        double VELOCITY = 1350;
        // --- INIT PHASE ---
        // WHY: Centralized init in RobotHardware sets motor directions, encoder modes, IMU orientation, etc.
        // TODO(STUDENTS): Confirm IMU orientation & Motor names in RobotHardware.init()
        robot.init();

        while (opModeInInit()) {

            // Student Note: Pre‑start check — rotate robot by hand; heading should change.
            // "Vision: Ready (AprilTag)" means camera + processor initialized.
            telemetry.addData("Status", "Hardware Initialized");
            telemetry.addData("Vision", "Ready (AprilTag)");
            telemetry.addData("Mode", "INIT");
//            telemetry.addData("Obelisk", robot.hasObeliskMotif() ? String.format("%s (ID %s)",
//                    robot.getObeliskMotif(), robot.getObeliskTagId()) : "–");
            telemetry.addData("Heading", "%4.0f", robot.getHeading());
            telemetry.update();
        }

        // Wait for driver to press START on Driver Station
        waitForStart();
        if (isStopRequested()) return;
        // --- TELEOP LOOP ---
        while (opModeIsActive()) {
            //shooting mechanism


            switch (turretState) {
                case Start:
                    if (gamepad1.right_trigger > 0.5) {
                        statusMsg = "RUNNING AUTO SHOOTER";
                        robot.turretVelocity(VELOCITY);
                        turretState = TwoPlayerRC.TurretState.HoldWait;
                    }
                    break;


                case HoldWait:
                    if (robot.getVelocity() >= VELOCITY-20) {
                        robot.intakePower(-1);
                        turretTimer.reset();
                        turretState = TwoPlayerRC.TurretState.Fire;
                    }
                    break;


                case Fire:
                    // Fire until manually stopped or velocity drops
                    if (isTimeUp(4.0)) {
                        turretState = TwoPlayerRC.TurretState.End;
                    }
                    break;


                case End:
                    statusMsg = "";
                    robot.turretVelocity(0);
                    robot.intakePower(0);
                    turretState = TwoPlayerRC.TurretState.Start;
                    break;
            }

            //############ CONTROLS ##############
            //kill switch for auto shooter
            if (gamepad1.y && turretState != turretState.Start) {
                robot.setIntakeServo(1);
                robot.turretVelocity(0);
                robot.intakePower(0);
                turretState = TurretState.Start;
                //telementry
                turretTimer.reset();
            }

                // Manual Intake Power (Trigger/X)
                if (gamepad1.left_trigger > .5) {
                    robot.intakePower(-1);
                } else if (gamepad1.x) {
                    robot.intakePower(1);
                } else {
                    robot.intakePower(0);
                }

                // Manual Intake Servo (A Button)
                if (gamepad1.a) {
                    robot.setIntakeServo(0);
                } else {
                    robot.setIntakeServo(1);
                }
                // Manual Hood  0 = High arch
                if (gamepad1.b) {

                    servoOn = !servoOn;

                }
                if(servoOn){
                    robot.setHoodPositions(1);
                }
                else {
                    robot.setHoodPositions(.8);
                }
                // manual turret shooter #note same as slow mode camera
                if(gamepad1.left_bumper){
                    robot.turretVelocity(VELOCITY);
                }
                else{
                    robot.turretVelocity(0);
                }
            }
            // Keep vision fresh before using pose values each loop
            robot.updateAprilTagDetections();


            // Student Note: Hold LB for precision (slow) mode.
            boolean slow = gamepad1.left_bumper;
            double scale = slow ? 1 : .8;

            axial = -gamepad2.left_stick_y * scale;
            lateral = gamepad2.left_stick_x * scale;
            yaw = gamepad2.right_stick_x * scale;

            // --- Vision helpers for concise telemetry ---
            Integer goalId = robot.getGoalTagId();
            double range = robot.getGoalRangeIn();
            double bearing = robot.getGoalBearingDeg();
            double elevation = robot.getGoalElevationDeg();

            // Approx horizontal distance and aim-above-horizontal (camera pitched up 15°)
            double horiz = (Double.isNaN(range) || Double.isNaN(bearing))
                    ? Double.NaN
                    : range * Math.cos(Math.toRadians(bearing));
            double aimAboveHorizontal = (Double.isNaN(elevation) ? Double.NaN : (15.0 + elevation));

            // Driver Assist: hold RB to auto-drive toward the visible goal tag (range->drive, yaw->strafe, bearing->turn).
            boolean autoAssist = gamepad1.right_bumper;
            boolean didAuto = false;
            if (autoAssist) {
                robot.updateAprilTagDetections();
                didAuto = robot.autoDriveToGoalStep();
            }

            // Student Note: Field‑centric drive call (mixing happens in RobotHardware) unless auto applied.
            if (!didAuto) {
                robot.driveRobotCentric(axial, lateral, yaw);
            }
            // ############### Controls ################
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
            // ############### Camera Data ################
            telemetry.addData("Mode", slow ? "SLOW" : "NORMAL");
            telemetry.addData("Assist", autoAssist ? (didAuto ? "AUTO→TAG" : "NO TAG") : "MANUAL");
            telemetry.addData("Heading", "%4.0f°", robot.getHeading());
            telemetry.addData("Drive", "ax=%.2f  lat=%.2f  yaw=%.2f", axial, lateral, yaw);
            String motif = robot.hasObeliskMotif() ? String.format("%s (ID %s)", robot.getObeliskMotif(), robot.getObeliskTagId()) : "–";
            telemetry.addData("Pose", "rng=%.1f in  brg=%.1f°  elev=%.1f°", range, bearing, elevation);
            telemetry.addData("Aim", "horiz=%.1f in  aboveHoriz=%s",
                    horiz,
                    Double.isNaN(aimAboveHorizontal) ? "–" : String.format("%.1f°", aimAboveHorizontal));
            telemetry.addData("TagYaw", "%.1f°", robot.getTagYawDeg());
            // non tag code
            telemetry.addData("Controls", "Drive/Strafe: Left Stick | Turn: Right Stick");
            telemetry.addData("Inputs", "axial=%.2f   lateral=%.2f   yaw=%.2f", axial, lateral, yaw);
            // Optional: expose heading during tuning
            // telemetry.addData("Heading(rad)", robot.getHeadingRadians()); / add a getter in RobotHardware if desired
            //servo stuff
            telemetry.addData("Drive", "Left Stick");
            telemetry.addData("Turn", "Right Stick");

            telemetry.addData("Obelisk", motif);
            telemetry.addData("Goal", (goalId != null) ? goalId : "–");

            //telemetry.addData("Turret motor 1 speed",robot.getVelocityMotor1());
            //telemetry.addData("Turret motor 2 speed",robot.getVelocityMotor2());


            telemetry.update();

            sleep(50);


        }

    }




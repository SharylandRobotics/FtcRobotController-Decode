/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.team12397;

import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//april tag

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import java.util.List;

import static java.lang.Thread.sleep;
//lamoin code
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

/**
 * RobotHardware
 *
 * PURPOSE:
 * - Centralized hardware mapping and drivetrain helpers so OpModes stay clean.
 * - Provide both robot-Centric and Field-Centric drive utilities
 *
 * STUDENT CHECKLIST:
 * 1) TODO(STUDENTS): Update motor names to match your Robot Configuration
 * 2) TODO(STUDENTS): Verify IMU orientation (logo/USB direction) match your hub mount.
 * 3) (OPTIONAL) Set reversal directions to match your wiring/wheel mounting.
 *
 * SAFETY/DEBUG:
 * - If the robot drives "sideways" or heading feels wrong, verify IMU orientation + motor directions.
 * - Use telemetry to print axial/lateral/yaw and botHeading when tuning.
 */
public class RobotHardware {

    // We hold a reference to the active OpMode to access hardwareMap/telemetry safely
    private LinearOpMode myOpMode = null;

    // Drivetrain motors for a mecanum chassis
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

    // IMU is used for field-centric heading
    private IMU imu;
    // servo set up
    private Servo hood;

    public static final double MID_SERVO       =  0;
    // intake servo
    private Servo intakeServo;
    // intake motor
    private DcMotor intakeMotor;
    //turret motor
    private DcMotorEx turretMotor;
    private DcMotorEx turretMotor2;


    // camera
    //private AprilTagProcessor aprilTag;
    //private VisionPortal visionPortal;

    // Student Note: Camera pose (robot frame). +X forward, +Y left, +Z up (in).
    // Pitch +15° = camera looks UP 15°. Update if you remount the camera.
    // TODO(students): Measure real offsets when you rely on precise vision assists.
    private final Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, 15, 0, 0);

    private AprilTagProcessor aprilTag = null;

    private Integer obeliskTagId = null;
    private String obeliskMotif = null;

    private Integer goalTagId = null;
    private double goalRangeIn = Double.NaN;
    private double goalBearingDeg = Double.NaN;
    private double goalElevationDeg = Double.NaN;

    // Note: tagYawDeg is the TAG'S image rotation (not the robot's yaw). We apply this to lateral (strafe).
    private double tagYawDeg = Double.NaN;

    private static final double DESIRED_DISTANCE = 70.4; // camera-to-tag inches
    private static final double AXIAL_GAIN = 0.020; // rangeError -> axial (forward/back) speed
    private static final double LATERAL_GAIN = 0.02; // tagYawError -> lateral (strafe) speed
    private static final double YAW_GAIN = 0.010; // bearingError -> yaw (turn) speed
    public static final double MAX_AUTO_AXIAL = 0.50;
    public static final double MAX_AUTO_LATERAL = 0.50;
    public static final double MAX_AUTO_YAW = 0.30;
    static final double HEADING_THRESHOLD = 1.0;

    // Student Note: Calibrated intrinsics for 1280×800. Must match camera resolution.
    // TODO(students): Recalibrate or update values if resolution or lens changes.
    private static final double LENS_FX = 921.31;
    private static final double LENS_FY = 917.70;
    private static final double LENS_CX = 689.03;
    private static final double LENS_CY = 372.06;

    private boolean isObelisk(AprilTagDetection d) {
        String name = (d != null && d.metadata != null) ? d.metadata.name : "";
        return name != null && name.toLowerCase().contains("obelisk");
    }
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize hardware mappings and base motor/IMU configuration.
     * Call once from your OPMode before driving.
     */
    public void init() {
        // --- HARDWARE MAP NAMES ---
        // TODO(STUDENTS): These strings MUST match your Driver Station Robot Configuration.
        frontLeftDrive = myOpMode.hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = myOpMode.hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = myOpMode.hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = myOpMode.hardwareMap.get(DcMotor.class, "back_right_drive");
        //servo
        hood = myOpMode.hardwareMap.get(Servo.class, "hood_servo");
        //hood.setPosition(MID_SERVO);
        // second servo
        intakeServo = myOpMode.hardwareMap.get(Servo.class, "intake_servo");
        //intakeServo.setPosition(1);
        //intake motor
        intakeMotor = myOpMode.hardwareMap.get(DcMotor.class, "intake_motor");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //turret motor reverse?
        turretMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "turret_motor");
        turretMotor.setDirection(DcMotor.Direction.REVERSE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setVelocityPIDFCoefficients(100,1,2,1);

        turretMotor2 = myOpMode.hardwareMap.get(DcMotorEx.class, "turret_motor2");
        turretMotor2.setDirection(DcMotor.Direction.FORWARD);
        turretMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor2.setVelocityPIDFCoefficients(100,1,2,1);





        // --- IMU ORIENTATION ---
        // TODO(STUDENTS): Update if your Control/Expansion Hub is mounted differently.
        // The two enums MUST reflect the physical orientation of the REV Hub on the robot.
        // WHY: Field-centric depends on accurate yaw; wrong orientation => wrong heading rotations.
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,        // e.g., logo pointing up
                RevHubOrientationOnRobot.UsbFacingDirection.UP));  // e.g., USB ports towards front

        imu = myOpMode.hardwareMap.get(IMU.class, "imu"); // TODO(STUDENTS): confirm IMU name
        imu.initialize(parameters);

        // NOTE: Reset yaw at init so heading starts ~0 at OpMode start.
        // If you prefer "press A to zero heading", move this to your OpMode and bind to a button.
        imu.resetYaw();
        initAprilTag();
        // --- MOTOR DIRECTIONS ---
        // NOTE: these reversals are common for mecanum so "axial + lateral" maps correctly.
        // TODO(STUDENTS): If the robot strafes opposite or spins wrong, swap these directions.
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // --- ENCODER MODES ---
        // WHY: Reset once at init for a clean baseline; then RUN_USING_ENCODER for closed-loop speed control if needed.
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // NOTE: BRAKE helps with precise stopping; FLOAT cna feel smoother when coasting.
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        myOpMode.telemetry.addData("Status", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Robot-Centric drive (driver-relative): no IMU rotation applied.
     * @param axial     forward/backward (+forward)
     * @param lateral   left/right (+ right)
     * @param yaw       rotate CCW (+ left turn)
     */
    public void driveRobotCentric(double axial, double lateral, double yaw) {
        // WHY: Standard mecanum mixing (A + L + Y. etc.). Values may exceed |1|; we normalize below.
        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

        // Normalize so that the highest magnitude is 1.0, preserving ratios
        double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }
        // change max speed in hardware  for field and drive centric
        final double MAX_SPEED = 1; // 50% of full speed
        frontLeftPower  *= MAX_SPEED;
        frontRightPower *= MAX_SPEED;
        backLeftPower   *= MAX_SPEED;
        backRightPower  *= MAX_SPEED;

        setDrivePower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    /**
     * Field-Centric drive (field-relative): rotates driver inputs by -heading so forward is field-forward.
     * @param axial     forward/backward from stick
     * @param lateral   left/right from stick
     * @param yaw       rotation command
     */
    public void driveFieldCentric(double axial, double lateral, double yaw) {
        // NOTE: Heading is in radians; positive CCW. We rotate the input vector by -heading.
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Input rotation for field frame
        double lateralRotation = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
        double axialRotation = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);

        // WHY: Standard mecanum mixing (A + L + Y. etc.). Values may exceed |1|; we normalize below.
        double frontLeftPower  = axialRotation + lateralRotation + yaw;
        double frontRightPower = axialRotation - lateralRotation - yaw;
        double backLeftPower   = axialRotation - lateralRotation + yaw;
        double backRightPower  = axialRotation + lateralRotation - yaw;

        // Normalize so that the highest magnitude is 1.0, preserving ratios
        double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }
        // change max speed in hardware  for field and drive centric
        final double MAX_SPEED = 1; // 50% of full speed
        frontLeftPower  *= MAX_SPEED;
        frontRightPower *= MAX_SPEED;
        backLeftPower   *= MAX_SPEED;
        backRightPower  *= MAX_SPEED;

        setDrivePower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    /**
     * Low-level power application.
     * NOTE: No ramping here-add slew rate limiting on TeleOP if you want softer starts.
     */
    public void setDrivePower(double frontLeftWheel, double frontRightWheel, double backLeftWheel, double backRightWheel) {
        frontLeftDrive.setPower(frontLeftWheel);
        frontRightDrive.setPower(frontRightWheel);
        backLeftDrive.setPower(backLeftWheel);
        backRightDrive.setPower(backRightWheel);

    }
    //servos
    public void setHoodPositions(double position) {
        hood.setPosition(position);
    }
    public double getHoodPosition(){
        double x = hood.getPosition();
        return x;
    }
    public void setIntakeServo(double position){
        intakeServo.setPosition(position);
    }




    //turret
    public void turretVelocity(double velocity){
        turretMotor.setVelocity(velocity);
        turretMotor2.setVelocity(velocity);

    }
    public void turretPower(double power){
        turretMotor.setPower(power);
    }

    public double getVelocity(){
        return turretMotor.getVelocity();
    }
    // intake motor
    public void intakePower(double power){
        intakeMotor.setPower(power);
    }


// auto methods
// Inertial Measurement Unit (IMU) for heading and orientation


    private double headingError;

    private double targetHeading;

    private double axialSpeed;
    private double lateralSpeed;
    private double yawSpeed;
    private double frontLeftSpeed;
    private double backLeftSpeed;
    private double frontRightSpeed;
    private double backRightSpeed;
    private int frontLeftTarget;
    private int backLeftTarget;
    private int frontRightTarget;
    private int backRightTarget;

    // Drive geometry and encoder model (update if wheels/gearing change)
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.094;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    // Default speeds and proportional gains; HEADING_THRESHOLD in degrees
    public static final double AXIAL_SPEED = 0.4;
    public static final double LATERAL_SPEED = 0.4;
    public static final double YAW_SPEED = 1;
    //static final double HEADING_THRESHOLD = 1.0;

    static final double P_AXIAL_GAIN = 0.03;
    static final double P_LATERAL_GAIN = 0.03;
    static final double P_YAW_GAIN = 0.02;
    public void driveStraight(double maxAxialSpeed, double distance, double heading) {

        if (myOpMode.opModeIsActive()) {

            // Convert inches to encoder counts for straight motion
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            frontLeftTarget = frontLeftDrive.getCurrentPosition() + moveCounts;
            backLeftTarget = backLeftDrive.getCurrentPosition() + moveCounts;
            frontRightTarget = frontRightDrive.getCurrentPosition() + moveCounts;
            backRightTarget = backRightDrive.getCurrentPosition() + moveCounts;

            // Same target on all wheels → straight move
            frontLeftDrive.setTargetPosition(frontLeftTarget);
            backLeftDrive.setTargetPosition(backLeftTarget);
            frontRightDrive.setTargetPosition(frontRightTarget);
            backRightDrive.setTargetPosition(backRightTarget);

            // Use built-in position control to reach targets
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // RUN_TO_POSITION requires positive power magnitude
            axialSpeed = Math.abs(maxAxialSpeed);
            driveRobotCentric(axialSpeed, 0, 0);

            while (myOpMode.opModeIsActive() &&
                    (frontLeftDrive.isBusy() && backLeftDrive.isBusy() &&
                            frontRightDrive.isBusy() && backRightDrive.isBusy())) {

                // Proportional yaw correction to stay on heading while driving
                yawSpeed = getSteeringCorrection(heading, P_AXIAL_GAIN);

                // Invert correction when backing up
                if (distance < 0)
                    yawSpeed *= -1.0;

                driveRobotCentric(axialSpeed, 0, yawSpeed);

                myOpMode.telemetry.addData("Motion", "Drive Straight");
                myOpMode.telemetry.addData("Target Pos FL:BL:FR:BR", "%7d:%7d:%7d:%7d",
                        frontLeftTarget,  backLeftTarget, frontRightTarget, backRightTarget);
                myOpMode.telemetry.addData("Actual Pos FL:BL:FR:BR","%7d:%7d:%7d:%7d",
                        frontLeftDrive.getCurrentPosition(), backLeftDrive.getCurrentPosition(),
                        frontRightDrive.getCurrentPosition(), backRightDrive.getCurrentPosition());
                myOpMode.telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f",
                        targetHeading, getHeading());
                myOpMode.telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f",
                        headingError, yawSpeed);
                myOpMode.telemetry.addData("Wheel Speeds FL:BL:FR:BR", "%5.2f : %5.2f : %5.2f : %5.2f",
                        frontLeftSpeed, backLeftSpeed, frontRightSpeed, backRightSpeed);
                myOpMode.telemetry.update();
            }

            driveRobotCentric(0, 0, 0);

            // Restore TeleOp run mode after completing the move
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    //positive straif right negative straif left
    public void straif(double maxAxialSpeed, double distance, double heading) {

        if (myOpMode.opModeIsActive()) {

            // Convert inches to encoder counts for straight motion
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            frontLeftTarget = frontLeftDrive.getCurrentPosition() + moveCounts;
            backLeftTarget = backLeftDrive.getCurrentPosition() - moveCounts;
            frontRightTarget = frontRightDrive.getCurrentPosition() - moveCounts;
            backRightTarget = backRightDrive.getCurrentPosition() + moveCounts;

            // Same target on all wheels → straight move
            frontLeftDrive.setTargetPosition(frontLeftTarget);
            backLeftDrive.setTargetPosition(backLeftTarget);
            frontRightDrive.setTargetPosition(frontRightTarget);
            backRightDrive.setTargetPosition(backRightTarget);

            // Use built-in position control to reach targets
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // RUN_TO_POSITION requires positive power magnitude
            axialSpeed = Math.abs(maxAxialSpeed);
            driveRobotCentric(axialSpeed, 0, 0);

            while (myOpMode.opModeIsActive() &&
                    (frontLeftDrive.isBusy() && backLeftDrive.isBusy() &&
                            frontRightDrive.isBusy() && backRightDrive.isBusy())) {

                // Proportional yaw correction to stay on heading while driving
                yawSpeed = getSteeringCorrection(heading, P_AXIAL_GAIN);

                // Invert correction when backing up
                if (distance < 0)
                    yawSpeed *= -1.0;

                driveRobotCentric(axialSpeed, 0, yawSpeed);

                myOpMode.telemetry.addData("Motion", "Drive Straight");
                myOpMode.telemetry.addData("Target Pos FL:BL:FR:BR", "%7d:%7d:%7d:%7d",
                        frontLeftTarget,  backLeftTarget, frontRightTarget, backRightTarget);
                myOpMode.telemetry.addData("Actual Pos FL:BL:FR:BR","%7d:%7d:%7d:%7d",
                        frontLeftDrive.getCurrentPosition(), backLeftDrive.getCurrentPosition(),
                        frontRightDrive.getCurrentPosition(), backRightDrive.getCurrentPosition());
                myOpMode.telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f",
                        targetHeading, getHeading());
                myOpMode.telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f",
                        headingError, yawSpeed);
                myOpMode.telemetry.addData("Wheel Speeds FL:BL:FR:BR", "%5.2f : %5.2f : %5.2f : %5.2f",
                        frontLeftSpeed, backLeftSpeed, frontRightSpeed, backRightSpeed);
                myOpMode.telemetry.update();
            }

            driveRobotCentric(0, 0, 0);

            // Restore TeleOp run mode after completing the move
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    // Turn in place to target heading using proportional control
    public void turnToHeading(double maxYawSpeed, double heading) {

        getSteeringCorrection(heading, P_YAW_GAIN);

        while (myOpMode.opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Limit turn power to requested maximum
            yawSpeed = getSteeringCorrection(heading, P_YAW_GAIN);

            yawSpeed = Range.clip(yawSpeed, -maxYawSpeed, maxYawSpeed);

            driveRobotCentric(0, 0, yawSpeed);

            myOpMode.telemetry.addData("Motion", "Turning");
            myOpMode.telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f",
                    targetHeading, getHeading());
            myOpMode.telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f",
                    headingError, yawSpeed);
            myOpMode.telemetry.addData("Wheel Speeds FL:BL:FR:BR", "%5.2f : %5.2f : %5.2f : %5.2f",
                    frontLeftSpeed, backLeftSpeed, frontRightSpeed, backRightSpeed);
            myOpMode.telemetry.update();
        }

        // Stop turning and recenter outputs
        driveRobotCentric(0, 0, 0);
    }

    // Maintain heading for a fixed time (sec) to let the robot settle
    public void holdHeading(double maxYawSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        while (myOpMode.opModeIsActive() && (holdTimer.time() < holdTime)) {

            yawSpeed = getSteeringCorrection(heading, P_YAW_GAIN);

            yawSpeed = Range.clip(yawSpeed, -maxYawSpeed, maxYawSpeed);

            driveRobotCentric(0, 0, yawSpeed);

            myOpMode.telemetry.addData("Motion", "Hold Heading");
            myOpMode.telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f",
                    targetHeading, getHeading());
            myOpMode.telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f",
                    headingError, yawSpeed);
            myOpMode.telemetry.addData("Wheel Speeds FL:BL:FR:BR", "%5.2f : %5.2f : %5.2f : %5.2f",
                    frontLeftSpeed, backLeftSpeed, frontRightSpeed, backRightSpeed);
            myOpMode.telemetry.update();
        }

        driveRobotCentric(0, 0, 0);
    }

    // Return clipped turn command from normalized heading error (-180, 180]
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;

        headingError = -targetHeading + getHeading();

        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        return Range.clip(headingError * proportionalGain, -1.0, 1.0);
    }

    // Basic robot-centric drive: movement relative to robot orientation


    // Apply calculated power values to each motor
    public void autoSetDrivePower(double frontLeftWheel, double frontRightWheel, double backLeftWheel, double backRightWheel) {
        frontLeftSpeed = frontLeftWheel;
        backLeftSpeed = backLeftWheel;
        frontRightSpeed = frontRightWheel;
        backRightSpeed = backRightWheel;

        frontLeftDrive.setPower(frontLeftWheel);
        frontRightDrive.setPower(frontRightWheel);
        backLeftDrive.setPower(backLeftWheel);
        backRightDrive.setPower(backRightWheel);
    }

    // Return current IMU yaw (rotation around vertical axis) in degrees
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    // april tag
    private void initAprilTag() {
        // Student Note: Build AprilTag processor with camera pose + intrinsics; start Dashboard stream.
        // TODO(students): If Dashboard video looks flipped, add a display‑only flip in a processor.
        AprilTagProcessor.Builder tagBuilder = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(LENS_FX, LENS_FY, LENS_CX, LENS_CY);

        aprilTag = tagBuilder.build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280, 720))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();

        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
        visionPortal.setProcessorEnabled(aprilTag, true);
    }

    // Student Note: Never navigate on Obelisk tags—only latch which one we saw.
    // Goal targeting: keep current goal if visible; else take first non‑Obelisk.
    // Clears pose when no goal is visible to avoid stale data.
    // TODO(students): Add a "target lock" button if drivers want sticky targeting.
    public void updateAprilTagDetections() {
        if (aprilTag == null) return;

        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections == null || detections.isEmpty()) {
            goalTagId = null;
            goalRangeIn = Double.NaN;
            goalBearingDeg = Double.NaN;
            goalElevationDeg = Double.NaN;
            tagYawDeg = Double.NaN;
            return;
        }

        if (obeliskMotif == null) {
            for (AprilTagDetection d : detections) {
                if (isObelisk(d)) {
                    obeliskMotif = (d.metadata != null) ? d.metadata.name : "Obelisk";
                    obeliskTagId = d.id;
                    break;
                }
            }
        }

        AprilTagDetection chosen = null;
        if (goalTagId != null) {
            for (AprilTagDetection d : detections) {
                if (!isObelisk(d) && d.id == goalTagId) {
                    chosen = d;
                    break;
                }
            }
        }

        if (chosen == null) {
            for (AprilTagDetection d : detections) {
                if (!isObelisk(d)) {
                    chosen = d;
                    break;
                }
            }
        }

        if (chosen != null) {
            goalTagId = chosen.id;
            if (chosen.ftcPose != null) {
                goalRangeIn = chosen.ftcPose.range;
                goalBearingDeg = chosen.ftcPose.bearing;
                goalElevationDeg = chosen.ftcPose.elevation;
                tagYawDeg = chosen.ftcPose.yaw;
            } else {
                goalRangeIn = Double.NaN;
                goalBearingDeg = Double.NaN;
                goalElevationDeg = Double.NaN;
                tagYawDeg = Double.NaN;
            }
        } else {
            goalTagId = null;
            goalRangeIn = Double.NaN;
            goalBearingDeg = Double.NaN;
            goalElevationDeg = Double.NaN;
            tagYawDeg = Double.NaN;
        }
    }

    public boolean hasObeliskMotif() {return obeliskMotif != null; }

    public Integer getObeliskTagId() { return obeliskTagId; }

    public String getObeliskMotif() { return obeliskMotif; }

    public Integer getGoalTagId() { return goalTagId; }

    public double getGoalRangeIn() { return goalRangeIn; }

    public double getGoalBearingDeg() { return goalBearingDeg; }

    public double getGoalElevationDeg() { return  goalElevationDeg; }

    public double getTagYawDeg() { return tagYawDeg; }

    public void resetObeliskMotif() {
        obeliskMotif = null;
        obeliskTagId = null;
    }
    public boolean autoDriveToGoalStep() {
        if (Double.isNaN(goalRangeIn) || Double.isNaN(goalBearingDeg)) {
            return false;
        }
        double rangeError = (goalRangeIn - DESIRED_DISTANCE)*0;


        double headingError =  goalBearingDeg;
        double yawError = (Double.isNaN(tagYawDeg) ? 0.0 : tagYawDeg);

        double axial = Range.clip(rangeError * AXIAL_GAIN, -MAX_AUTO_AXIAL,   MAX_AUTO_AXIAL);
        //changed yaw error to range error lateral
        double lateral = Range.clip(yawError * LATERAL_GAIN, -MAX_AUTO_LATERAL,  MAX_AUTO_LATERAL)*0;
        double yaw = Range.clip(-headingError * YAW_GAIN, -MAX_AUTO_YAW, MAX_AUTO_YAW);

        driveRobotCentric(axial, lateral, yaw);
        return true;
    }






}


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

package org.firstinspires.ftc.team00000;

import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Shared hardware + robot services layer for Team00000.
 *
 * <p>This class is intentionally reusable from both TeleOp and Autonomous so students can learn
 * one API surface for drivetrain, shooter, and vision-assisted behaviors.
 */
public class RobotHardware {

    // Owning OpMode context (hardwareMap + telemetry + opMode lifecycle).
    private final LinearOpMode myOpMode;

    // Mecanum drivetrain.
    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;

    // Dual-wheel shooter in velocity-control mode.
    private DcMotorEx topShooter;
    private DcMotorEx bottomShooter;

    // Continuous-rotation servos for transfer and stopper mechanisms.
    private CRServo leftStopper;
    private CRServo rightStopper;
    private CRServo frontLeftTransfer;
    private CRServo frontRightTransfer;
    private CRServo backLeftTransfer;
    private CRServo backRightTransfer;

    // IMU yaw is used for field-centric drive and heading hold.
    private IMU imu = null;

    // Cached control/telemetry values.
    private double headingError;
    private double targetHeading;
    private double yawSpeed;
    private double frontLeftSpeed;
    private double backLeftSpeed;
    private double frontRightSpeed;
    private double backRightSpeed;

    // Camera pose in robot coordinates (+X forward, +Y left, +Z up).
    // Update if the camera mount geometry changes.
    private final Position cameraPosition = new Position(DistanceUnit.INCH,
            0, -2.34, 16, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, 8, 0, 0);

    // AprilTag detection state and selected target.
    private AprilTagProcessor aprilTag = null;
    private Integer obeliskTagId = null;
    private String obeliskMotif = null;
    private Integer goalTagId = null;
    private double goalRangeIn = Double.NaN;
    private double goalBearingDeg = Double.NaN;
    private double goalElevationDeg = Double.NaN;

    // Tag yaw (degrees) from FTC pose estimate, used for lateral alignment.
    private double tagYawDeg = Double.NaN;

    // AprilTag assist tuning (units: inches/degrees).
    private static final double DESIRED_DISTANCE = 140.0; // Camera-to-tag range target.
    private static double DESIRED_YAW_DEG = 0;
    private static double DESIRED_BEARING_DEG = 0;
    private static final double AXIAL_GAIN = 0.020; // range error -> forward/back command
    private static final double LATERAL_GAIN = 0.015; // tag yaw error -> strafe command
    private static final double YAW_GAIN = 0.015; // bearing error -> turn command
    public static final double MAX_AUTO_AXIAL = 0.90;
    public static final double MAX_AUTO_LATERAL = 0.90;
    public static final double MAX_AUTO_YAW = 0.85;
    public static final double HEADING_THRESHOLD = 1.0;
    public static final double TOLERANCE_TICKS = 10.0;

    // Shooter target used by readiness checks and telemetry.
    private double shooterTargetTicksPerSec = 0.0;

    // Cached shooter auto-model state (range filter + voltage compensation).
    private double shooterAutoFilteredRangeIn = Double.NaN;
    private long shooterAutoLastValidRangeMs = 0;
    private double shooterAutoVoltageComp = 1.0;
    private double shooterAutoBaseCmd = 0.0;
    private double shooterAutoBatteryVoltage = Double.NaN;

    // Camera intrinsics in pixels; re-calibrate if camera/resolution changes.
    private static final double LENS_FX = 921.31;
    private static final double LENS_FY = 917.70;
    private static final double LENS_CX = 689.03;
    private static final double LENS_CY = 372.06;

    // Obelisk tags are landmarks, not scoring targets for this assist behavior.
    private boolean isObelisk(AprilTagDetection d) {
        String name = (d != null && d.metadata != null) ? d.metadata.name : "";
        return name != null && name.toLowerCase().contains("obelisk");
    }

    // Drive encoder model for inches conversion.
    // Update if wheel diameter, gearing, or motor model changes.
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.094;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    /**
     * @param opmode owning OpMode, used for hardware map access and lifecycle checks
     */
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Maps all configured hardware devices and applies baseline run modes/directions.
     *
     * <p>This should be called once in each OpMode before start.
     */
    public void init() {

        frontLeftDrive = myOpMode.hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = myOpMode.hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = myOpMode.hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = myOpMode.hardwareMap.get(DcMotor.class, "back_right_drive");

        topShooter = myOpMode.hardwareMap.get(DcMotorEx.class, "top_shooter");
        bottomShooter = myOpMode.hardwareMap.get(DcMotorEx.class, "bottom_shooter");

        leftStopper = myOpMode.hardwareMap.get(CRServo.class, "left_stopper");
        rightStopper = myOpMode.hardwareMap.get(CRServo.class, "right_stopper");
        frontLeftTransfer = myOpMode.hardwareMap.get(CRServo.class, "front_left_transfer");
        frontRightTransfer = myOpMode.hardwareMap.get(CRServo.class, "front_right_transfer");
        backLeftTransfer = myOpMode.hardwareMap.get(CRServo.class, "back_left_transfer");
        backRightTransfer = myOpMode.hardwareMap.get(CRServo.class, "back_right_transfer");

        // IMU orientation must match the Control Hub mounting orientation.
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        topShooter.setDirection(DcMotor.Direction.FORWARD);
        bottomShooter.setDirection(DcMotor.Direction.FORWARD);

        leftStopper.setDirection(DcMotorSimple.Direction.FORWARD);
        rightStopper.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftTransfer.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftTransfer.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightTransfer.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Zero yaw so heading control starts from a known reference.
        imu.resetYaw();
        resetShooterAutoModel(0.0);
        initAprilTag();
    }

    /**
     * Drives a field-relative vector for a fixed distance while holding heading.
     *
     * @param maxSpeed wheel power limit [0..1]
     * @param distance distance in inches (negative moves opposite travelHeading)
     * @param travelHeading field heading in degrees (0 = +X field forward)
     * @param robotHeading heading to hold in degrees
     */
    public void driveOmni(double maxSpeed, double distance, double travelHeading, double robotHeading) {

        if (myOpMode.opModeIsActive()) {

            double angleRadians = Math.toRadians(travelHeading);

            double axialDistance = distance * Math.cos(angleRadians);
            double lateralDistance = distance * Math.sin(angleRadians);

            int axialMoveCounts = (int)(axialDistance * COUNTS_PER_INCH);
            int lateralMoveCounts = (int)(lateralDistance * COUNTS_PER_INCH);

            int frontLeftTarget = frontLeftDrive.getCurrentPosition() + axialMoveCounts + lateralMoveCounts;
            int backLeftTarget = backLeftDrive.getCurrentPosition() + axialMoveCounts - lateralMoveCounts;
            int frontRightTarget = frontRightDrive.getCurrentPosition() + axialMoveCounts - lateralMoveCounts;
            int backRightTarget = backRightDrive.getCurrentPosition() + axialMoveCounts + lateralMoveCounts;

            frontLeftDrive.setTargetPosition(frontLeftTarget);
            backLeftDrive.setTargetPosition(backLeftTarget);
            frontRightDrive.setTargetPosition(frontRightTarget);
            backRightDrive.setTargetPosition(backRightTarget);

            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            double totalDistance = Math.hypot(axialDistance, lateralDistance);
            double axialFraction = 0;
            double lateralFraction = 0;
            if (totalDistance > 0.001) {
                axialFraction = axialDistance / totalDistance;
                lateralFraction = lateralDistance / totalDistance;
            }

            driveFieldCentric(axialFraction * maxSpeed, lateralFraction * maxSpeed, 0);

            while (myOpMode.opModeIsActive() &&
                    (frontLeftDrive.isBusy() || backLeftDrive.isBusy() ||
                    frontRightDrive.isBusy() || backRightDrive.isBusy())) {

                double leftFrontError = Math.abs(frontLeftTarget - frontLeftDrive.getCurrentPosition());
                double leftBackError = Math.abs(backLeftTarget - backLeftDrive.getCurrentPosition());
                double rightFrontError = Math.abs(frontRightTarget - frontRightDrive.getCurrentPosition());
                double rightBackError = Math.abs(backRightTarget - backRightDrive.getCurrentPosition());

                // Closed-loop heading correction while RUN_TO_POSITION is active.
                double maxError =
                        Math.max(Math.max(leftFrontError, leftBackError),
                                Math.max(rightFrontError, rightBackError));

                yawSpeed = getSteeringCorrection(robotHeading, YAW_GAIN);

                if (maxError <= TOLERANCE_TICKS) {
                    break;
                }
                if (distance < 0)
                    yawSpeed *= -1.0;

                driveFieldCentric(axialFraction * maxSpeed, lateralFraction * maxSpeed, yawSpeed);

                myOpMode.telemetry.addData("Motion", "Drive Straight");
                myOpMode.telemetry.addData("Target Pos FL:BL:FR:BR", "%7d:%7d:%7d:%7d",
                        frontLeftTarget, backLeftTarget, frontRightTarget, backRightTarget);
                myOpMode.telemetry.addData("Actual Pos FL:BL:FR:BR","%7d:%7d:%7d:%7d",
                        frontLeftDrive.getCurrentPosition(), backLeftDrive.getCurrentPosition(),
                        frontRightDrive.getCurrentPosition(), backRightDrive.getCurrentPosition());
                myOpMode.telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f",
                        targetHeading, getHeading());
                myOpMode.telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f",
                        headingError, yawSpeed);
                myOpMode.telemetry.addData("Wheel Speeds FL:BL:FR:BR", "%5.2f : %5.2f : %5.2f : %5.2f",
                        frontLeftSpeed, backLeftSpeed, frontRightSpeed, backRightSpeed);
                updateAprilTagDetections();
                myOpMode.telemetry.update();
            }

            driveFieldCentric(0, 0, 0);

            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Turns in place until heading error is within {@link #HEADING_THRESHOLD}.
     *
     * @param maxYawSpeed absolute turn power cap [0..1]
     * @param heading target heading in degrees
     */
    public void turnToHeading(double maxYawSpeed, double heading) {

        getSteeringCorrection(heading, YAW_GAIN);

        // Turn in place until heading error is within tolerance.
        while (myOpMode.opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            yawSpeed = getSteeringCorrection(heading, YAW_GAIN);

            yawSpeed = Range.clip(yawSpeed, -maxYawSpeed, maxYawSpeed);

            driveFieldCentric(0, 0, yawSpeed);

            myOpMode.telemetry.addData("Motion", "Turning");
            myOpMode.telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f",
                    targetHeading, getHeading());
            myOpMode.telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f",
                    headingError, yawSpeed);
            myOpMode.telemetry.addData("Wheel Speeds FL:BL:FR:BR", "%5.2f : %5.2f : %5.2f : %5.2f",
                    frontLeftSpeed, backLeftSpeed, frontRightSpeed, backRightSpeed);
            updateAprilTagDetections();
            myOpMode.telemetry.update();
        }

        driveFieldCentric(0, 0, 0);
    }

    /**
     * Holds heading for a timed window, typically after a move.
     *
     * @param maxYawSpeed absolute turn power cap [0..1]
     * @param heading target heading in degrees
     * @param holdTime hold duration in seconds
     */
    public void holdHeading(double maxYawSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // Hold heading for a fixed duration (typically after a move).
        while (myOpMode.opModeIsActive() && (holdTimer.time() < holdTime)) {

            yawSpeed = getSteeringCorrection(heading, YAW_GAIN);

            yawSpeed = Range.clip(yawSpeed, -maxYawSpeed, maxYawSpeed);

            driveFieldCentric(0, 0, yawSpeed);

            myOpMode.telemetry.addData("Motion", "Hold Heading");
            myOpMode.telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f",
                    targetHeading, getHeading());
            myOpMode.telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f",
                    headingError, yawSpeed);
            myOpMode.telemetry.addData("Wheel Speeds FL:BL:FR:BR", "%5.2f : %5.2f : %5.2f : %5.2f",
                    frontLeftSpeed, backLeftSpeed, frontRightSpeed, backRightSpeed);
            updateAprilTagDetections();
            myOpMode.telemetry.update();
        }

        driveFieldCentric(0, 0, 0);
    }

    /**
     * Computes proportional turn correction from current yaw error.
     *
     * <p>Positive return value commands CCW turn. Error is wrapped to [-180, 180).
     *
     * @param desiredHeading target heading in degrees
     * @param proportionalGain P gain applied to wrapped heading error
     * @return clipped turn command in [-1, 1]
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;

        headingError = -targetHeading + getHeading();

        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        return Range.clip(headingError * proportionalGain, -1.0, 1.0);
    }

    /**
     * Robot-centric mecanum drive command.
     *
     * @param axial forward/back command (+forward)
     * @param lateral left/right strafe command (+left)
     * @param yaw turn command (+CCW)
     */
    public void driveRobotCentric(double axial, double lateral, double yaw) {
        // Standard mecanum inverse kinematics in robot frame.
        double frontLeftPower = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower = axial - lateral + yaw;
        double backRightPower = axial + lateral - yaw;

        double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        setDrivePower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    /**
     * Field-centric mecanum drive command.
     *
     * <p>Input translation is rotated by robot yaw so drivers can push "up field" regardless of
     * robot orientation.
     *
     * @param axial forward/back command in field frame (+forward)
     * @param lateral left/right command in field frame (+left)
     * @param yaw turn command (+CCW)
     */
    public void driveFieldCentric(double axial, double lateral, double yaw) {
        // Rotate field-frame command into robot frame before mecanum mixing.
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double lateralRotation = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
        double axialRotation = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);

        double frontLeftPower= axialRotation + lateralRotation + yaw;
        double frontRightPower = axialRotation - lateralRotation - yaw;
        double backLeftPower = axialRotation - lateralRotation + yaw;
        double backRightPower = axialRotation + lateralRotation - yaw;

        double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        setDrivePower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    /**
     * Directly applies wheel powers after normalization by caller.
     */
    public void setDrivePower(double frontLeftWheel, double frontRightWheel, double backLeftWheel, double backRightWheel) {
        frontLeftSpeed = frontLeftWheel;
        backLeftSpeed = backLeftWheel;
        frontRightSpeed = frontRightWheel;
        backRightSpeed = backRightWheel;

        frontLeftDrive.setPower(frontLeftWheel);
        frontRightDrive.setPower(frontRightWheel);
        backLeftDrive.setPower(backLeftWheel);
        backRightDrive.setPower(backRightWheel);
    }

    /**
     * Commands shooter target speed as a percent of a supplied max RPM model.
     *
     * @param power command percent [0..1]
     * @param maxRpm max RPM reference used for conversion to ticks/sec
     */
    public void setShooterVelocityPercent(double power, double maxRpm) {
        power = Range.clip(power, 0.0, 1.0);
        final double targetRpm = power * maxRpm;
        final double ticksPerRev = topShooter.getMotorType().getTicksPerRev();
        shooterTargetTicksPerSec = targetRpm * ticksPerRev / 60.0;

        // Command both shooter motors to the same target velocity.
        topShooter.setVelocity(shooterTargetTicksPerSec);
        bottomShooter.setVelocity(shooterTargetTicksPerSec);
    }

    /**
     * Configures REV velocity PIDF terms for shooter motors.
     *
     * <p>kF is derived from modeled max speed and adjusted by battery voltage.
     */
    public void configureShooterVelocityPIDFForMaxRpm(double maxRpm, double kP, double kI, double kD) {
        final double ticksPerRev = topShooter.getMotorType().getTicksPerRev();
        final double maxTicksPerSec = (maxRpm * ticksPerRev) / 60.0;
        final double nominalVoltage = 12.0;
        double batteryVoltage = getBatteryVoltage();

        // Feedforward scaled so full output maps to modeled max velocity.
        double kF = 32767.0 / Math.max(1.0, maxTicksPerSec);
        if (!Double.isNaN(batteryVoltage) && batteryVoltage > 1.0) {
            kF *= (nominalVoltage / batteryVoltage);
        }

        topShooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        bottomShooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);
    }

    /**
     * @return model-based max shooter speed in ticks/second from motor metadata
     */
    public double getShooterMaxTicksPerSec() {
        // Derived from configured motor model metadata.
        final double maxRpm = topShooter.getMotorType().getMaxRPM();
        final double ticksPerRev = topShooter.getMotorType().getTicksPerRev();
        return (maxRpm * ticksPerRev) / 60.0;
    }

    /**
     * @return current shooter target in ticks/second
     */
    public double getShooterTargetTicksPerSec() { return shooterTargetTicksPerSec; }

    /**
     * @return signed shooter velocity in ticks/second (use abs() for magnitude)
     */
    public double getShooterVelocityPerSecond() { return topShooter.getVelocity(); }

    /**
     * @return shooter velocity in RPM from encoder tick rate
     */
    public double getShooterVelocityRpm() {
        final double ticksPerRev = topShooter.getMotorType().getTicksPerRev();
        return topShooter.getVelocity() * 60.0 / ticksPerRev;
    }

    /**
     * @return shooter encoder position in ticks
     */
    public int getShooterEncoderPosition() {
        return topShooter.getCurrentPosition();
    }

    /**
     * @return shooter motor model ticks/revolution
     */
    public double getShooterEncoderTicksPerRevolution() {
        return topShooter.getMotorType().getTicksPerRev();
    }

    /**
     * @return shooter motor model max RPM
     */
    public double getShooterMaxRpm() {
        return topShooter.getMotorType().getMaxRPM();
    }

    /**
     * @return minimum positive hub voltage seen across all voltage sensors, or NaN if unavailable
     */
    public double getBatteryVoltage () {
        double min = Double.POSITIVE_INFINITY;
        for (VoltageSensor vs : myOpMode.hardwareMap.voltageSensor) {
            double v = vs.getVoltage();
            if (v > 0) min = Math.min(min, v);
        }
        return (min == Double.POSITIVE_INFINITY) ? Double.NaN : min;
    }

    /**
     * Resets cached shooter auto-model state.
     *
     * @param fallbackCmd command to use until valid range data is available
     */
    public void resetShooterAutoModel(double fallbackCmd) {
        shooterAutoFilteredRangeIn = Double.NaN;
        shooterAutoLastValidRangeMs = 0;
        shooterAutoVoltageComp = 1.0;
        shooterAutoBatteryVoltage = Double.NaN;
        shooterAutoBaseCmd = fallbackCmd;
    }

    /**
     * Updates the distance + voltage compensated shooter command model.
     *
     * <p>This is stateless from the caller's perspective but keeps internal cached filter state.
     * It is intentionally reusable from TeleOp and Autonomous.
     */
    public void updateShooterAutoModel(
            double rawRangeIn,
            long nowMs,
            double rangeAlpha,
            int rangeHoldMs,
            double referenceRangeIn,
            double referenceCmd,
            double cmdPerIn,
            double fallbackCmd,
            double minCmd,
            double maxCmd,
            double nominalVoltage,
            double voltageCompWeight,
            double voltageCompMin,
            double voltageCompMax,
            double trimCmd) {

        updateShooterAutoRangeFilter(rawRangeIn, nowMs, rangeAlpha, rangeHoldMs);
        shooterAutoBatteryVoltage = getBatteryVoltage();
        shooterAutoVoltageComp = computeShooterVoltageCompFactor(
                shooterAutoBatteryVoltage,
                nominalVoltage,
                voltageCompWeight,
                voltageCompMin,
                voltageCompMax);
        shooterAutoBaseCmd = computeShooterAutoBaseCommand(
                shooterAutoFilteredRangeIn,
                referenceRangeIn,
                referenceCmd,
                cmdPerIn,
                fallbackCmd,
                minCmd,
                maxCmd,
                shooterAutoVoltageComp,
                trimCmd);
    }

    private void updateShooterAutoRangeFilter(double rawRangeIn, long nowMs, double rangeAlpha, int rangeHoldMs) {
        if (Double.isFinite(rawRangeIn)) {
            double alpha = Range.clip(rangeAlpha, 0.0, 1.0);
            shooterAutoFilteredRangeIn = Double.isFinite(shooterAutoFilteredRangeIn)
                    ? alpha * rawRangeIn + (1.0 - alpha) * shooterAutoFilteredRangeIn
                    : rawRangeIn;
            shooterAutoLastValidRangeMs = nowMs;
            return;
        }
        if (shooterAutoLastValidRangeMs == 0 || nowMs - shooterAutoLastValidRangeMs > Math.max(0, rangeHoldMs)) {
            shooterAutoFilteredRangeIn = Double.NaN;
        }
    }

    /**
     * Computes a multiplicative feedforward correction from battery voltage.
     *
     * @return voltage compensation factor, clipped into [voltageCompMin, voltageCompMax]
     */
    public double computeShooterVoltageCompFactor(
            double batteryVoltage,
            double nominalVoltage,
            double voltageCompWeight,
            double voltageCompMin,
            double voltageCompMax) {

        double minComp = Math.min(voltageCompMin, voltageCompMax);
        double maxComp = Math.max(voltageCompMin, voltageCompMax);
        if (!Double.isFinite(batteryVoltage) || batteryVoltage <= 1.0) {
            return Range.clip(1.0, minComp, maxComp);
        }
        double nominal = (nominalVoltage > 1.0) ? nominalVoltage : 12.0;
        double weight = Range.clip(voltageCompWeight, 0.0, 2.0);
        double ratio = nominal / batteryVoltage;
        double comp = 1.0 + (ratio - 1.0) * weight;
        return Range.clip(comp, minComp, maxComp);
    }

    /**
     * Computes final base shooter command from distance model + voltage compensation + trim.
     *
     * @return clipped shooter command in [minCmd, maxCmd]
     */
    public double computeShooterAutoBaseCommand(
            double rangeIn,
            double referenceRangeIn,
            double referenceCmd,
            double cmdPerIn,
            double fallbackCmd,
            double minCmd,
            double maxCmd,
            double voltageCompFactor,
            double trimCmd) {

        double cmdMin = Math.min(minCmd, maxCmd);
        double cmdMax = Math.max(minCmd, maxCmd);
        double modeledBase = Double.isFinite(rangeIn)
                ? referenceCmd + (rangeIn - referenceRangeIn) * cmdPerIn
                : fallbackCmd;
        modeledBase = Range.clip(modeledBase, cmdMin, cmdMax);
        double cmd = modeledBase * voltageCompFactor + trimCmd;
        return Range.clip(cmd, cmdMin, cmdMax);
    }

    /**
     * @return filtered tag range in inches used by the shooter auto model
     */
    public double getShooterAutoFilteredRangeIn() { return shooterAutoFilteredRangeIn; }

    /**
     * @return latest voltage compensation factor applied by shooter auto model
     */
    public double getShooterAutoVoltageComp() { return shooterAutoVoltageComp; }

    /**
     * @return latest shooter base command computed by shooter auto model
     */
    public double getShooterAutoBaseCmd() { return shooterAutoBaseCmd; }

    /**
     * @return latest battery voltage sample used by shooter auto model
     */
    public double getShooterAutoBatteryVoltage() { return shooterAutoBatteryVoltage; }

    /**
     * Percent-based shooter speed readiness test against current velocity target.
     *
     * @param percentTolerance allowable error as fraction of target (e.g. 0.10 = +/-10%)
     */
    public boolean isShooterAtSpeedPercent(double percentTolerance) {
        double targetTicksPerSecond = getShooterTargetTicksPerSec();
        if (Math.abs(targetTicksPerSecond) <= 1.0) return false;

        double measuredTicksPerSecond = Math.abs(getShooterVelocityPerSecond());
        double toleranceTicksPerSecond = Math.abs(targetTicksPerSecond) * percentTolerance;
        return Math.abs(measuredTicksPerSecond - Math.abs(targetTicksPerSecond)) <= toleranceTicksPerSecond;
    }

    /**
     * Shooter readiness helper used by both TeleOp and Autonomous.
     *
     * <p>Returns true if either:
     * 1) percent-based tracking is within tolerance, or
     * 2) measured speed exceeds a minimum floor.
     */
    public boolean isShooterReady(double percentTolerance, double minTicksPerSecond) {
        if (Math.abs(getShooterTargetTicksPerSec()) <= 1.0) {
            return false;
        }
        if (isShooterAtSpeedPercent(percentTolerance)) {
            return true;
        }
        if (minTicksPerSecond > 0.0) {
            return Math.abs(getShooterVelocityPerSecond()) >= minTicksPerSecond;
        }
        return false;
    }

    /**
     * Sets both stopper CR servos to the same power.
     */
    public void setStopperPower(double power) {
        double p = Range.clip(power, -1.0, 1.0);
        leftStopper.setPower(p);
        rightStopper.setPower(p);
    }

    /**
     * Sets all transfer CR servos to the same power.
     */
    public void setTransferPower(double power) {
        double p = Range.clip(power, -1.0, 1.0);
        frontLeftTransfer.setPower(p);
        frontRightTransfer.setPower(p);
        backLeftTransfer.setPower(p);
        backRightTransfer.setPower(p);
    }

    /**
     * @return robot yaw heading in degrees from IMU
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    /**
     * Re-zeros yaw so current orientation becomes heading 0.
     */
    public void resetYaw() {
        if (imu != null) {
            imu.resetYaw();
        }
    }

    private void initAprilTag() {
        // Build processor using calibrated intrinsics/extrinsics.
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
                .setCameraResolution(new Size(1280, 800))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();

        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
        visionPortal.setProcessorEnabled(aprilTag, true);
    }

    /**
     * Refreshes AprilTag detections and selects a non-Obelisk goal target.
     *
     * <p>When no valid goal is visible, cached pose values are cleared to prevent stale control.
     */
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

                // Tag-specific offsets for asymmetric field placements.
                if (goalTagId == 20) {
                    DESIRED_YAW_DEG = 20;
                    DESIRED_BEARING_DEG = 0;
                } else if (goalTagId == 24) {
                    DESIRED_YAW_DEG = -20;
                    DESIRED_BEARING_DEG = 4;
                } else {
                    DESIRED_YAW_DEG = 0;
                    DESIRED_BEARING_DEG = 0;
                }
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

    /**
     * @return true once an Obelisk landmark has been identified
     */
    public boolean hasObeliskMotif() {return obeliskMotif != null; }

    /**
     * @return last detected Obelisk tag id, or null if unknown
     */
    public Integer getObeliskTagId() { return obeliskTagId; }

    /**
     * @return last detected Obelisk motif/name, or null if unknown
     */
    public String getObeliskMotif() { return obeliskMotif; }

    /**
     * @return selected goal tag id, or null if not visible
     */
    public Integer getGoalTagId() { return goalTagId; }

    /**
     * @return selected goal range in inches, or NaN if unavailable
     */
    public double getGoalRangeIn() { return goalRangeIn; }

    /**
     * @return selected goal bearing in degrees, or NaN if unavailable
     */
    public double getGoalBearingDeg() { return goalBearingDeg; }

    /**
     * @return selected goal elevation in degrees, or NaN if unavailable
     */
    public double getGoalElevationDeg() { return  goalElevationDeg; }

    /**
     * @return selected goal tag yaw in degrees, or NaN if unavailable
     */
    public double getTagYawDeg() { return tagYawDeg; }

    /**
     * Clears cached Obelisk landmark data so it can be reacquired.
     */
    public void resetObeliskMotif() {
        obeliskMotif = null;
        obeliskTagId = null;
    }

    /**
     * Executes one closed-loop assist step toward the selected goal tag.
     *
     * <p>Returns false when pose is not available or produced invalid commands.
     *
     * @return true if a valid assist command was applied
     */
    public boolean autoDriveToGoalStep() {
        if (!Double.isFinite(goalRangeIn) || !Double.isFinite(goalBearingDeg)) {
            return false;
        }
        double rangeError = (goalRangeIn - DESIRED_DISTANCE);
        double headingError =  goalBearingDeg - DESIRED_BEARING_DEG;
        double yawError = Double.isFinite(tagYawDeg) ? (tagYawDeg - DESIRED_YAW_DEG) : 0.0;

        // Sign conventions:
        //  +axial   -> drive forward
        //  +lateral -> strafe left
        //  +yaw     -> CCW rotation
        double axial = Range.clip(rangeError * AXIAL_GAIN, -MAX_AUTO_AXIAL,   MAX_AUTO_AXIAL);
        double lateral = Range.clip(yawError * LATERAL_GAIN, -MAX_AUTO_LATERAL,  MAX_AUTO_LATERAL);
        double yaw = Range.clip(-headingError * YAW_GAIN, -MAX_AUTO_YAW, MAX_AUTO_YAW);
        if (!Double.isFinite(axial) || !Double.isFinite(lateral) || !Double.isFinite(yaw)) {
            return false;
        }

        driveRobotCentric(axial, lateral, yaw);
        return true;
    }
}

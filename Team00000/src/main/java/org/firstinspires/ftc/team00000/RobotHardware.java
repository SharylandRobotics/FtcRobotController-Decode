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

// Shared hardware and service layer used by Team00000 OpModes.
public class RobotHardware {

    // Owning OpMode context.
    private final LinearOpMode myOpMode;

    // Mecanum drivetrain motors.
    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;

    // Shooter motors in velocity-control mode.
    private DcMotorEx topShooter;
    private DcMotorEx bottomShooter;

    // Continuous-rotation servos for transfer and stopper mechanisms.
    private CRServo leftStopper;
    private CRServo rightStopper;
    private CRServo frontLeftTransfer;
    private CRServo frontRightTransfer;
    private CRServo backLeftTransfer;
    private CRServo backRightTransfer;

    // IMU provides yaw for field-centric drive and heading control.
    private IMU imu = null;

    // Cached control and telemetry state.
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

    // AprilTag assist tuning (units: inches and degrees).
    private static final double DESIRED_DISTANCE = 140.0; // Camera-to-tag range target.
    private static double DESIRED_YAW_DEG = 0;
    private static double DESIRED_BEARING_DEG = 0;
    private static final double AXIAL_GAIN = 0.020; // rangeError -> forward/back command
    private static final double LATERAL_GAIN = 0.015; // yawError -> strafe command
    private static final double YAW_GAIN = 0.010; // bearingError -> turn command
    public static final double MAX_AUTO_AXIAL = 0.90;
    public static final double MAX_AUTO_LATERAL = 0.90;
    public static final double MAX_AUTO_YAW = 0.70;
    public static final double HEADING_THRESHOLD = 1.0;
    public static final double TOLERANCE_TICKS = 10.0;

    // Shooter velocity target used for telemetry and at-speed logic.
    private double shooterTargetTicksPerSec = 0.0;

    // Camera intrinsics in pixels; calibrate per camera/resolution.
    private static final double LENS_FX = 921.31;
    private static final double LENS_FY = 917.70;
    private static final double LENS_CX = 689.03;
    private static final double LENS_CY = 372.06;

    // Obelisk tags are non-goal landmarks for this autonomous/assist logic.
    private boolean isObelisk(AprilTagDetection d) {
        String name = (d != null && d.metadata != null) ? d.metadata.name : "";
        return name != null && name.toLowerCase().contains("obelisk");
    }

    // Drive encoder model for distance conversion (inches).
    // Update if wheel diameter, gearing, or motor model changes.
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.094;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

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
        initAprilTag();
    }

    // Drive a field-relative vector for a fixed distance while holding heading.
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

    // Positive output commands CCW turn. Error is wrapped to [-180, 180).
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;

        headingError = -targetHeading + getHeading();

        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        return Range.clip(headingError * proportionalGain, -1.0, 1.0);
    }

    public void driveRobotCentric(double axial, double lateral, double yaw) {
        // Robot-centric mecanum mix.
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

    public void driveFieldCentric(double axial, double lateral, double yaw) {
        // Field-centric mecanum mix (rotate input by current robot yaw).
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

    public void setShooterVelocityPercent(double power, double maxRpm) {
        power = Range.clip(power, 0.0, 1.0);
        final double targetRpm = power * maxRpm;
        final double ticksPerRev = topShooter.getMotorType().getTicksPerRev();
        shooterTargetTicksPerSec = targetRpm * ticksPerRev / 60.0;

        // Command both shooter motors to the same target velocity.
        topShooter.setVelocity(shooterTargetTicksPerSec);
        bottomShooter.setVelocity(shooterTargetTicksPerSec);
    }

    public void configureShooterVelocityPIDFForMaxRpm(double maxRpm, double kP, double kI, double kD) {
        final double ticksPerRev = topShooter.getMotorType().getTicksPerRev();
        final double maxTicksPerSec = (maxRpm * ticksPerRev) / 60.0;

        // Feedforward scaled so full output maps to modeled max velocity.
        final double kF = 32767.0 / Math.max(1.0, maxTicksPerSec);

        topShooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        bottomShooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);
    }

    public double getShooterMaxTicksPerSec() {
        // Derived from configured motor model metadata.
        final double maxRpm = topShooter.getMotorType().getMaxRPM();
        final double ticksPerRev = topShooter.getMotorType().getTicksPerRev();
        return (maxRpm * ticksPerRev) / 60.0;
    }

    public double getShooterTargetTicksPerSec() { return shooterTargetTicksPerSec; }

    // Signed velocity in ticks/sec; apply abs() for magnitude where needed.
    public double getShooterVelocityPerSecond() { return topShooter.getVelocity(); }

    public double getShooterVelocityRpm() {
        final double ticksPerRev = topShooter.getMotorType().getTicksPerRev();
        return topShooter.getVelocity() * 60.0 / ticksPerRev;
    }

    // Shooter encoder/model diagnostics.
    public int getShooterEncoderPosition() {
        return topShooter.getCurrentPosition();
    }

    public double getShooterEncoderTicksPerRevolution() {
        return topShooter.getMotorType().getTicksPerRev();
    }

    public double getShooterMaxRpm() {
        return topShooter.getMotorType().getMaxRPM();
    }

    public double getBatteryVoltage () {
        double min = Double.POSITIVE_INFINITY;
        for (VoltageSensor vs : myOpMode.hardwareMap.voltageSensor) {
            double v = vs.getVoltage();
            if (v > 0) min = Math.min(min, v);
        }
        return (min == Double.POSITIVE_INFINITY) ? Double.NaN : min;
    }

    public boolean isShooterAtSpeedPercent(double percentTolerance) {
        double targetTicksPerSecond = getShooterTargetTicksPerSec();
        if (Math.abs(targetTicksPerSecond) <= 1.0) return false;

        double measuredTicksPerSecond = Math.abs(getShooterVelocityPerSecond());
        double toleranceTicksPerSecond = Math.abs(targetTicksPerSecond) * percentTolerance;
        return Math.abs(measuredTicksPerSecond - Math.abs(targetTicksPerSecond)) <= toleranceTicksPerSecond;
    }

    public void setStopperPower(double power) {
        double p = Range.clip(power, -1.0, 1.0);
        leftStopper.setPower(p);
        rightStopper.setPower(p);
    }

    public void setTransferPower(double power) {
        double p = Range.clip(power, -1.0, 1.0);
        frontLeftTransfer.setPower(p);
        frontRightTransfer.setPower(p);
        backLeftTransfer.setPower(p);
        backRightTransfer.setPower(p);
    }

    // Current yaw heading in degrees.
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    // Re-zero yaw so current orientation becomes heading 0.
    public void resetYaw() {
        if (imu != null) {
            imu.resetYaw();
        }
    }

    private void initAprilTag() {
        // Build AprilTag processor from known camera extrinsics and intrinsics.
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

    // Refresh detections and select a non-Obelisk goal tag.
    // Clear pose values when no valid target is visible to avoid stale control data.
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

                // Apply tag-specific offsets for asymmetric field placements.
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

    // Execute one closed-loop assist step toward the selected goal tag.
    public boolean autoDriveToGoalStep() {
        if (Double.isNaN(goalRangeIn) || Double.isNaN(goalBearingDeg)) {
            return false;
        }
        double rangeError = (goalRangeIn - DESIRED_DISTANCE);
        double headingError =  goalBearingDeg - DESIRED_BEARING_DEG;
        double yawError = tagYawDeg - DESIRED_YAW_DEG;

        // Sign conventions:
        //  +axial   -> drive forward
        //  +lateral -> strafe left
        //  +yaw     -> CCW rotation
        double axial = Range.clip(rangeError * AXIAL_GAIN, -MAX_AUTO_AXIAL,   MAX_AUTO_AXIAL);
        double lateral = Range.clip(yawError * LATERAL_GAIN, -MAX_AUTO_LATERAL,  MAX_AUTO_LATERAL);
        double yaw = Range.clip(-headingError * YAW_GAIN, -MAX_AUTO_YAW, MAX_AUTO_YAW);

        driveRobotCentric(axial, lateral, yaw);
        return true;
    }
}

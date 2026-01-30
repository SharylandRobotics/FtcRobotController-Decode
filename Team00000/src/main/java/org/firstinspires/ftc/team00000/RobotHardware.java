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

// Student Notes: Hardware wrapper ("robot API") for drive, IMU, and AprilTag vision.
// Keep comments concise; use TODOs to guide improvements.
public class RobotHardware {

    private final LinearOpMode myOpMode;

    // Student Note: Mecanum drive motors. If motion is reversed, fix motor directions or wiring.
    // TODO(students): Verify these names match your RC configuration.
    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;

    private DcMotorEx topShooter;
    private DcMotorEx bottomShooter;

    private CRServo leftStopper;
    private CRServo rightStopper;
    private CRServo frontLeftTransfer;
    private CRServo frontRightTransfer;
    private CRServo backLeftTransfer;
    private CRServo backRightTransfer;

    // Student Note: IMU provides yaw (heading) for field‑centric drive and turns.
    // TODO(students): If heading seems rotated, check hub orientation in init().
    private IMU imu = null;

    private double headingError;

    private double targetHeading;

    private double yawSpeed;
    private double frontLeftSpeed;
    private double backLeftSpeed;
    private double frontRightSpeed;
    private double backRightSpeed;
    private int frontLeftTarget;
    private int backLeftTarget;
    private int frontRightTarget;
    private int backRightTarget;

    // Student Note: Camera pose (robot frame). +X forward, +Y left, +Z up (in).
    // Pitch +8° = camera looks UP 8°. Update if you remount the camera.
    // TODO(students): Measure real offsets when you rely on precise vision assists.
    private final Position cameraPosition = new Position(DistanceUnit.INCH,
            0, -2.34, 16, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, 8, 0, 0);

    private AprilTagProcessor aprilTag = null;

    private Integer obeliskTagId = null;
    private String obeliskMotif = null;

    private Integer goalTagId = null;
    private double goalRangeIn = Double.NaN;
    private double goalBearingDeg = Double.NaN;
    private double goalElevationDeg = Double.NaN;

    // Note: tagYawDeg is the TAG'S image rotation (not the robot's yaw). We apply this to lateral (strafe).
    private double tagYawDeg = Double.NaN;

    private static final double DESIRED_DISTANCE = 120.0; // camera-to-tag inches
    private static final double DESIRED_BEARING_DEG = 44.37;
    private static final double AXIAL_GAIN = 0.020; // rangeError -> axial (forward/back) speed
    private static final double LATERAL_GAIN = 0.015; // tagYawError -> lateral (strafe) speed
    private static final double YAW_GAIN = 0.010; // bearingError -> yaw (turn) speed
    public static final double MAX_AUTO_AXIAL = 0.90;
    public static final double MAX_AUTO_LATERAL = 0.90;
    public static final double MAX_AUTO_YAW = 0.70;
    public static final double HEADING_THRESHOLD = 1.0;
    public static final double TOLERANCE_TICKS = 10.0;

    private double shooterTargetTicksPerSec = 0.0;

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

    // Student Note: Encoder model. COUNTS_PER_INCH converts inches to encoder ticks.
    // TODO(students): Update if wheel size/gearing/encoders change.
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

        // Student Note: Control Hub mounting directions for correct IMU yaw.
        // TODO(students): If yaw sign/drift looks wrong, verify these settings.
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        topShooter.setDirection(DcMotor.Direction.REVERSE);
        bottomShooter.setDirection(DcMotor.Direction.REVERSE);

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

        // Student Note: Zero heading at init so 0° is the starting direction.
        imu.resetYaw();
        initAprilTag();
    }

    public void driveOmni(double maxSpeed, double distance, double travelHeading, double robotHeading) {

        if (myOpMode.opModeIsActive()) {

            double angleRadians = Math.toRadians(travelHeading);

            double axialDistance = distance * Math.cos(angleRadians);
            double lateralDistance = distance * Math.sin(angleRadians);

            int axialMoveCounts = (int)(axialDistance * COUNTS_PER_INCH);
            int lateralMoveCounts = (int)(lateralDistance * COUNTS_PER_INCH);

            frontLeftTarget = frontLeftDrive.getCurrentPosition() + axialMoveCounts + lateralMoveCounts;
            backLeftTarget = backLeftDrive.getCurrentPosition() + axialMoveCounts - lateralMoveCounts;
            frontRightTarget = frontRightDrive.getCurrentPosition() + axialMoveCounts - lateralMoveCounts;
            backRightTarget = backRightDrive.getCurrentPosition() + axialMoveCounts + lateralMoveCounts;

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

            // Student Note: Encoder straight drive with P‑turn correction to hold heading.
            // TODO(students): Tune P_AXIAL_GAIN if it wiggles or under‑corrects.
            while (myOpMode.opModeIsActive() &&
                    (frontLeftDrive.isBusy() && backLeftDrive.isBusy() &&
                    frontRightDrive.isBusy() && backRightDrive.isBusy())) {

                double leftFrontError = Math.abs(frontLeftTarget - frontLeftDrive.getCurrentPosition());
                double leftBackError = Math.abs(backLeftTarget - backLeftDrive.getCurrentPosition());
                double rightFrontError = Math.abs(frontRightTarget - frontRightDrive.getCurrentPosition());
                double rightBackError = Math.abs(backRightTarget - backRightDrive.getCurrentPosition());

                // Find the max distance among the four wheels
                double maxError =
                        Math.max(Math.max(leftFrontError, leftBackError),
                                Math.max(rightFrontError, rightBackError));

                yawSpeed = getSteeringCorrection(robotHeading, AXIAL_GAIN);

                if (maxError <= TOLERANCE_TICKS) {
                    break;
                }
                if (distance < 0)
                    yawSpeed *= -1.0;

                driveFieldCentric(axialFraction * maxSpeed, lateralFraction * maxSpeed, yawSpeed);

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

        // Student Note: Turn‑in‑place until heading error is small.
        // TODO(students): Tune P_YAW_GAIN for snappier or smoother turns.
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

        // Student Note: Briefly hold heading to let the robot settle after a move.
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

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;

        headingError = -targetHeading + getHeading();

        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        return Range.clip(headingError * proportionalGain, -1.0, 1.0);
    }

    public void driveRobotCentric(double axial, double lateral, double yaw) {
        // Student Note: Robot‑centric (relative to robot). No heading compensation here.
        // TODO(students): Add input deadbands or expo curves if sticks feel too touchy.
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
        // Student Note: Field‑centric (relative to field). Rotate sticks by IMU yaw.
        // TODO(students): If "up" isn't downfield, check IMU orientation & motor directions.
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
        final double ticksPerRev = topShooter.getMotorType().getTicksPerRev();
        final double targetRpm = power * maxRpm;
        shooterTargetTicksPerSec = targetRpm * ticksPerRev / 60.0;

        topShooter.setVelocity(shooterTargetTicksPerSec);
        bottomShooter.setVelocity(shooterTargetTicksPerSec);
    }

    public void configureShooterVelocityPidfForMaxRpm(double maxRpm, double kP, double kI, double kD) {
        final double ticksPerRev = topShooter.getMotorType().getTicksPerRev();
        final double maxTicksPerSec = (maxRpm * ticksPerRev) / 60.0;
        final double kF = 32767.0 / Math.max(1.0, maxTicksPerSec);

        topShooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        bottomShooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);
    }

    public double rpmToTicksPerSec(double rpm) {
        final double ticksPerRev = topShooter.getMotorType().getTicksPerRev();
        return (rpm * ticksPerRev) / 60.0;
    }
    public double getShooterTargetTicksPerSec() { return shooterTargetTicksPerSec; }
    public double getTopShooterTicksPerSec() { return Math.abs(topShooter.getVelocity()); }
    public double getBottomShooterTicksPerSec() { return Math.abs(bottomShooter.getVelocity()); }
    public double getTopShooterRpm() {
        final double ticksPerRev = topShooter.getMotorType().getTicksPerRev();
        return Math.abs(topShooter.getVelocity() * 60.0 / ticksPerRev);
    }
    public double getBottomShooterRpm() {
        final double ticksPerRev = bottomShooter.getMotorType().getTicksPerRev();
        return Math.abs(bottomShooter.getVelocity() * 60.0 / ticksPerRev);
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

    // Student Note: Convenience — current yaw (degrees) from the IMU.
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

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
                .setCameraResolution(new Size(1280, 800))
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
        double rangeError = (goalRangeIn - DESIRED_DISTANCE);
        double headingError =  goalBearingDeg;
        double yawError = (Double.isNaN(tagYawDeg) ? 0.0 : tagYawDeg);

        double axial = Range.clip(rangeError * AXIAL_GAIN, -MAX_AUTO_AXIAL,   MAX_AUTO_AXIAL);
        double lateral = Range.clip(yawError * LATERAL_GAIN, -MAX_AUTO_LATERAL,  MAX_AUTO_LATERAL);
        double yaw = Range.clip(-headingError * YAW_GAIN, -MAX_AUTO_YAW, MAX_AUTO_YAW);

        driveRobotCentric(axial, lateral, yaw);
        return true;
    }
}

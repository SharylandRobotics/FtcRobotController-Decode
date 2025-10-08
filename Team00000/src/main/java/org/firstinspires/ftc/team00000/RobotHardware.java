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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

// Student notes:
// • This class sets up drive motors, the IMU (heading), and AprilTags
// • Keep big decisions in your OpModes; this file provides small helpers
// • If you change motors or wheel size, re‑tune the constants below
// • Later we can make speeds/gains adjustable with FTC Dashboard
// • Used by both TeleOp and Autonomous OpModes to access robot functions
// • TODO (student): Update names, wheel size, and IMU orientation for YOUR robot before testing.

public class RobotHardware {

    // OpMode that owns hardwareMap/telemetry
    private final LinearOpMode myOpMode;

    // Drive motors (mecanum). Names must match the configuration.
    // Left side is reversed in init() so +axial drives forward.
    private DcMotorEx frontLeftDrive;
    private DcMotorEx backLeftDrive;
    private DcMotorEx frontRightDrive;
    private DcMotorEx backRightDrive;

    // IMU for yaw (robot heading). Make sure hub orientation matches the real robot.
    private IMU imu = null;

    // AprilTag camera (optional). Built in initAprilTag().
    // Only call camera methods after initAprilTag() has run.
    private VisionPortal visionPortal;
    public AprilTagProcessor aprilTag;
    public AprilTagDetection desiredTag;

    // Live state used for telemetry/debug each loop.
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

    // TODO (student): Set this to your motor's ticks-per-rev (e.g., 383.6 for goBILDA 312, 537.7 for 5202/435 RPM).
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    // TODO (student): If you have external gearing, set this to motor->wheel gear ratio ( <1 for speed-up, >1 for reduction ).
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    // TODO (student): Measure your wheel diameter (inches) tread-to-tread and update this value.
    static final double WHEEL_DIAMETER_INCHES = 4.094;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    // Default speed limits and simple P gains (no I/D).
    // HEADING_THRESHOLD is degrees of allowable error before we stop turning.
    // TODO (student): Cap speeds for your driver preference and battery health.
    public static final double AXIAL_SPEED = 0.5;
    public static final double LATERAL_SPEED = 0.5;
    public static final double YAW_SPEED = 0.3;
    static final double HEADING_THRESHOLD = 1.0;
    public final double DESIRED_DISTANCE = 12.0;

    // TODO (student): Tune these P gains to reduce drift/overshoot (start small, increase slowly).
    public static final double AXIAL_GAIN = 0.02;
    public static final double LATERAL_GAIN = 0.015;
    public static final double YAW_GAIN = 0.01;

    // Save a handle to the OpMode.
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    // Set up motors, encoders, IMU, and optional camera exposure.
    public void init() {

        // Mecanum drive motors: Front Left, Back Left, Front Right, Back Right
        // TODO (student): Change **all four** motor names above to match your configuration (Robot Controller app).
        frontLeftDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "front_left_drive");
        backLeftDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "back_left_drive");
        frontRightDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "front_right_drive");
        backRightDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "back_right_drive");

        // Hub orientation must match the robot.
        // TODO (student): Update hub orientation if your Control/Expansion Hub is mounted differently.
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        // Reverse left motors so all wheels drive forward consistently
        // TODO (student): If your robot drives backward when pushing forward, swap which side is REVERSE.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Reset encoders so RUN_TO_POSITION works predictably.
        // TODO (student): If you are NOT using encoders for TeleOp, you can keep RUN_WITHOUT_ENCODER later.
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Apply braking when motors receive zero power for stability
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TeleOp default: RUN_USING_ENCODER. Use RUN_WITHOUT_ENCODER for raw power.
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define 0° heading at init.
        // TODO (student): If your starting field heading is not 0°, call imu.resetYaw() at the start of your OpMode instead.
        imu.resetYaw();

        // If camera is running, set exposure/gain.
        setManualExposure(6, 250);
    }

    // Drive a distance (in) and hold a heading (deg) with RUN_TO_POSITION.
    public void driveStraight(double maxAxialSpeed, double distance, double heading) {

        // Loop runs until all motors reach their encoder targets or the OpMode stops
        if (myOpMode.opModeIsActive()) {

            // Convert inches → encoder counts
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            frontLeftTarget = frontLeftDrive.getCurrentPosition() + moveCounts;
            backLeftTarget = backLeftDrive.getCurrentPosition() + moveCounts;
            frontRightTarget = frontRightDrive.getCurrentPosition() + moveCounts;
            backRightTarget = backRightDrive.getCurrentPosition() + moveCounts;

            // Same target for all wheels = straight line
            frontLeftDrive.setTargetPosition(frontLeftTarget);
            backLeftDrive.setTargetPosition(backLeftTarget);
            frontRightDrive.setTargetPosition(frontRightTarget);
            backRightDrive.setTargetPosition(backRightTarget);

            // Use built-in position control to reach targets
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // RUN_TO_POSITION needs positive power; direction comes from targets
            axialSpeed = Math.abs(maxAxialSpeed);
            driveRobotCentric(axialSpeed, 0, 0);

            while (myOpMode.opModeIsActive() &&
                    (frontLeftDrive.isBusy() && backLeftDrive.isBusy() &&
                    frontRightDrive.isBusy() && backRightDrive.isBusy())) {

                // Small yaw correction to stay on heading
                yawSpeed = getSteeringCorrection(heading, AXIAL_GAIN);

                // Flip correction when moving backward
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

            // Restore RUN_USING_ENCODER when done
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // Turn in place to a target heading (P control).
    public void turnToHeading(double maxYawSpeed, double heading) {

        // Compute first correction
        getSteeringCorrection(heading, YAW_GAIN);

        // Keep turning until the heading error is smaller than the threshold
        while (myOpMode.opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Limit turn power to requested maximum
            yawSpeed = getSteeringCorrection(heading, YAW_GAIN);

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

    // Hold a heading for a set time.
    public void holdHeading(double maxYawSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // Keeps correcting yaw until the timer runs out
        while (myOpMode.opModeIsActive() && (holdTimer.time() < holdTime)) {

            yawSpeed = getSteeringCorrection(heading, YAW_GAIN);

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

    // Return turn power from heading error (deg). Error range is (-180, 180].
    // Used by drive and turn functions to keep the robot facing the correct direction
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;

        headingError = -targetHeading + getHeading();

        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        return Range.clip(headingError * proportionalGain, -1.0, 1.0);
    }

    // Robot-centric drive: sticks are relative to the robot.
    public void driveRobotCentric(double axial, double lateral, double yaw) {

        double frontLeftPower = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower = axial - lateral + yaw;
        double backRightPower = axial + lateral - yaw;

        // Normalize so no value exceeds 1.0
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

    // Field-centric drive: rotate stick input by current yaw so "up" = field forward.
    public void driveFieldCentric(double axial, double lateral, double yaw) {

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate joystick frame by -yaw
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

    // Send power to each motor and record values for telemetry.
    public void setDrivePower(double frontLeftWheel, double frontRightWheel, double backLeftWheel, double backRightWheel) {
        // TODO (student): Add a master scale or safety limit here if your robot is too fast in the pit.
        frontLeftSpeed = frontLeftWheel;
        backLeftSpeed = backLeftWheel;
        frontRightSpeed = frontRightWheel;
        backRightSpeed = backRightWheel;

        frontLeftDrive.setPower(frontLeftWheel);
        frontRightDrive.setPower(frontRightWheel);
        backLeftDrive.setPower(backLeftWheel);
        backRightDrive.setPower(backRightWheel);
    }
    // Build AprilTag pipeline and open the camera stream.
    // Call this once before using AprilTag detections
    public void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder()
                // TODO (student): Replace intrinsics with your camera's calibration values (fx, fy, cx, cy).
                .setLensIntrinsics(921.31, 917.70, 689.03, 372.06)
                .build();

        // TODO (student): Adjust decimation (1–3). Lower = more range, higher = more speed.
        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                // TODO (student): Choose a resolution that your Hub can handle reliably (e.g., 640x480, 800x600).
                .setCameraResolution(new android.util.Size(1280,800))
                .build();
    }

    // Set camera exposure/gain if portal exists and OpMode is running.
    // Helps the camera see better under bright lights (optional tweak)
    private void setManualExposure(int exposureMS, int gain) {
        // TODO (student): If your field is dim/bright, tweak exposureMS and gain; or remove to use auto-exposure.
        if (visionPortal == null) return;

        if (!myOpMode.isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
        }
    }

    // Current yaw (deg), CCW positive.
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}

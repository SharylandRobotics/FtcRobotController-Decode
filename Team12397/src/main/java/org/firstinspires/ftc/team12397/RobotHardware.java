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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;
//april tag

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

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

    //turret motor
    private DcMotor turretMotor;
    // camera
    //private AprilTagProcessor aprilTag;
    //private VisionPortal visionPortal;


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
        hood.setPosition(MID_SERVO);
        //turret motor reverse?
        turretMotor = myOpMode.hardwareMap.get(DcMotor.class, "turret_motor");
        turretMotor.setDirection(DcMotor.Direction.REVERSE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //april tag
        /*
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "webcam 1"))
                .addProcessor(aprilTag)
                .build();
        */





        // --- IMU ORIENTATION ---
        // TODO(STUDENTS): Update if your Control/Expansion Hub is mounted differently.
        // The two enums MUST reflect the physical orientation of the REV Hub on the robot.
        // WHY: Field-centric depends on accurate yaw; wrong orientation => wrong heading rotations.
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,        // e.g., logo pointing up
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));  // e.g., USB ports towards front

        imu = myOpMode.hardwareMap.get(IMU.class, "imu"); // TODO(STUDENTS): confirm IMU name
        imu.initialize(parameters);

        // NOTE: Reset yaw at init so heading starts ~0 at OpMode start.
        // If you prefer "press A to zero heading", move this to your OpMode and bind to a button.
        imu.resetYaw();

        // --- MOTOR DIRECTIONS ---
        // NOTE: these reversals are common for mecanum so "axial + lateral" maps correctly.
        // TODO(STUDENTS): If the robot strafes opposite or spins wrong, swap these directions.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
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
    //servo
    public void setHoodPositions(double position) {
        hood.setPosition(position);
    }
    //turret
    public void turretPower(double power){
        turretMotor.setPower(power);
    }
    // april tag
  /*
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

*/
}


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

package org.firstinspires.ftc.team13580;

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

import java.util.ArrayList;
import java.util.Arrays;
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
    private DcMotor intakeDrive;
    public DcMotorEx outtakeDrive;

    private CRServo kicker;
    private CRServo kickerLeft;



    // Student Note: IMU provides yaw (heading) for field‑centric drive and turns.
    // TODO(students): If heading seems rotated, check hub orientation in init().
    private IMU imu = null;

    private double headingError;

    private double targetHeading;

    private double axialSpeed;
    private double lateralSpeed;
    private double yawSpeed;
    private double frontLeftSpeed;
    private double backLeftSpeed;
    private double frontRightSpeed;
    private double backRightSpeed;
    private double intakeSpeed;
    private double outtakeSpeed;
    private int frontLeftTarget;
    private int backLeftTarget;
    private int frontRightTarget;
    private int backRightTarget;

    // Student Note: Camera pose (robot frame). +X forward, +Y left, +Z up (in).
    // Pitch +15° = camera looks UP 15°. Update if you remount the camera.
    // TODO(students): Measure real offsets when you rely on precise vision assists.
   private final Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
   private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 15, 0, 0);
    private AprilTagProcessor aprilTag = null;
    private Integer obeliskTagId = null;
    private String obeliskMotif = null;

    private Integer goalTagId = null;
    private double goalRangeIn = Double.NaN;
    private double goalBearingDeg = Double.NaN;
    private double goalElevationDeg = Double.NaN;

    // Note: tagYawDeg is the TAG'S image rotation (not the robot's yaw). We apply this to lateral (strafe).
    private double tagYawDeg = Double.NaN;

    private static final double DESIRED_DISTANCE = 24.0; // camera-to-tag inches
    private static final double AXIAL_GAIN = 0.020; // rangeError -> axial (forward/back) speed
    private static final double LATERAL_GAIN = 0.015; // tagYawError -> lateral (strafe) speed
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

    // Student Note: Encoder model. COUNTS_PER_INCH converts inches to encoder ticks.
    // TODO(students): Update if wheel size/gearing/encoders change.
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.094;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);


    public RobotHardware(LinearOpMode opMode) { myOpMode = opMode; }

    public void init() {

        frontLeftDrive = myOpMode.hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = myOpMode.hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = myOpMode.hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = myOpMode.hardwareMap.get(DcMotor.class, "back_right_drive");

        intakeDrive = myOpMode.hardwareMap.get(DcMotor.class, "intake_drive");
        outtakeDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "outtake_drive");

        kicker = myOpMode.hardwareMap.get(CRServo.class, "kicker");
        kickerLeft = myOpMode.hardwareMap.get(CRServo.class, "kicker_left");

                // Student Note: Control Hub mounting directions for correct IMU yaw.
        // TODO(students): If yaw sign/drift looks wrong, verify these settings.
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        intakeDrive.setDirection(DcMotor.Direction.REVERSE  );
        outtakeDrive.setDirection(DcMotor.Direction.FORWARD);


        outtakeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        outtakeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtakeDrive.setVelocityPIDFCoefficients(50, 1, 3, 1);

        // Student Note: Zero heading at init so 0° is the starting direction.
        imu.resetYaw();
        initAprilTag();
    }

    public void driveStraight(double maxAxialSpeed, double distance, double heading) {

        if (myOpMode.opModeIsActive()) {

            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            frontLeftTarget = frontLeftDrive.getCurrentPosition() + moveCounts;
            backLeftTarget = backLeftDrive.getCurrentPosition() + moveCounts;
            frontRightTarget = frontRightDrive.getCurrentPosition() + moveCounts;
            backRightTarget = backRightDrive.getCurrentPosition() + moveCounts;

            frontLeftDrive.setTargetPosition(frontLeftTarget);
            backLeftDrive.setTargetPosition(backLeftTarget);
            frontRightDrive.setTargetPosition(frontRightTarget);
            backRightDrive.setTargetPosition(backRightTarget);

            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            axialSpeed = Math.abs(maxAxialSpeed);
            driveRobotCentric(axialSpeed, 0, 0);

            // Student Note: Encoder straight drive with P‑turn correction to hold heading.
            // TODO(students): Tune P_AXIAL_GAIN if it wiggles or under‑corrects.
            while (myOpMode.opModeIsActive() &&
                    (frontLeftDrive.isBusy() && backLeftDrive.isBusy() &&
                    frontRightDrive.isBusy() && backRightDrive.isBusy())) {

                yawSpeed = getSteeringCorrection(heading, AXIAL_GAIN);

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
                //updateAprilTagDetections();
                myOpMode.telemetry.update();
            }

            driveRobotCentric(0, 0, 0);


        }
    }

    public void turnToHeading(double maxYawSpeed, double heading) {

        getSteeringCorrection(heading, YAW_GAIN);

        // Student Note: Turn‑in‑place until heading error is small.
        // TODO(students): Tune P_YAW_GAIN for snappier or smoother turns.
        while (myOpMode.opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

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
            //updateAprilTagDetections();
            myOpMode.telemetry.update();
        }

        driveRobotCentric(0, 0, 0);
    }

    public void holdHeading(double maxYawSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // Student Note: Briefly hold heading to let the robot settle after a move.
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
            //updateAprilTagDetections();
            myOpMode.telemetry.update();
        }

        driveRobotCentric(0, 0, 0);
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

    public void setIntakePower(double intakeWheel) {
        intakeSpeed = intakeWheel;
        intakeDrive.setPower(intakeWheel);
    }

    public void setOuttakePower(double outtakeWheel) {
        outtakeSpeed = outtakeWheel;
        outtakeDrive.setPower(outtakeWheel);
    }

    public void setOuttakeVelocity(int ticks){
        outtakeDrive.setVelocity(ticks);
    }

    // Student Note: Convenience — current yaw (degrees) from the IMU.
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void driveEncoder(double speed, double leftFrontInches, double leftBackInches, double rightFrontInches, double rightBackInches){
        // drives only while myOpMode is active
        if(myOpMode.opModeIsActive()){


            //determine new target position
            int leftFrontTarget = frontLeftDrive.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            int leftBackTarget = backLeftDrive.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            int rightFrontTarget = frontRightDrive.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            int rightBackTarget = backRightDrive.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);

            frontLeftDrive.setTargetPosition(leftFrontTarget - 1);
            backLeftDrive.setTargetPosition(leftBackTarget - 1 );
            frontRightDrive.setTargetPosition(rightFrontTarget - 1);
            backRightDrive.setTargetPosition(rightBackTarget - 1 );

            //turn on RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            setDrivePower(Math.abs(speed), Math.abs(speed), Math.abs(speed), Math.abs(speed));

            while ((myOpMode.opModeIsActive() &&
                    (frontLeftDrive.isBusy() && backLeftDrive.isBusy() &&
                            frontRightDrive.isBusy() && backRightDrive.isBusy()))){

                //display it for driver

                myOpMode.telemetry.addData("Running to ", " %7d :%7d :%7d :%7d",
                        leftFrontTarget, leftBackTarget, rightFrontTarget, rightBackTarget);
                myOpMode.telemetry.addData("Currently at ", "%7d ;%7d :%7d :%7d",
                        frontLeftDrive.getCurrentPosition(), backLeftDrive.getCurrentPosition(),
                        frontRightDrive.getCurrentPosition(), backRightDrive.getCurrentPosition());
                myOpMode.telemetry.update();
            }

            setDrivePower(0, 0, 0, 0 );
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            myOpMode.sleep(200); //optional pause after each move
        }
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
    // TODO(students): Add a "target lock" btton if drivers want sticky targeting.
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

    private ArrayList<double[]> velDataPoints = new ArrayList<>(Arrays.asList(
            new double[]{34, 1110},
            new double[]{50, 1160},
            new double[]{85, 1330},
            new double[]{105, 1470},
            new double[]{151, 1770}
    ));
    public double getCalculatedVelocity(double distance){
        double returnVal = 500;
        int maxIndex = velDataPoints.size()-1;
        double[] slopeList = new double[maxIndex];
        for (int i=0; i<maxIndex; i++){
            slopeList[i] = (
                    (velDataPoints.get(i+1)[1] - velDataPoints.get(i)[1]) /
                            (velDataPoints.get(i+1)[0] - velDataPoints.get(i)[0])
            );
        }
        double velZeroIntercept = velDataPoints.get(0)[1] - slopeList[0]*velDataPoints.get(0)[0];

        for (int i=0; i<slopeList.length; i++){
            if (i == 0){
                if (distance >= 0 && distance <= velDataPoints.get(0)[0]){
                    returnVal = velZeroIntercept + distance*slopeList[0];
                }
            } else if (distance > velDataPoints.get(i)[0] && distance <= velDataPoints.get(i+1)[0]){
                myOpMode.telemetry.addData("Bottom Reference: ", velDataPoints.get(i)[0] +", " +velDataPoints.get(i)[1]);
                myOpMode.telemetry.addData("Slope Reference: ", slopeList[i]);
                returnVal = velDataPoints.get(i)[1] + (distance-velDataPoints.get(i)[0])*slopeList[i];
            }
        }

        return returnVal;
    }

    public double getFloorDistance(){
        return Math.sqrt(Math.pow(getGoalRangeIn(), 2) - Math.pow(16,2));
    }



    ;


    public double getGoalBearingDeg() { return goalBearingDeg; }

    public double getGoalElevationDeg() { return  goalElevationDeg; }

    public double getTagYawDeg() { return tagYawDeg; }

    public void resetObeliskMotif() {
        obeliskMotif = null;
        obeliskTagId = null;
    }
    public void setKickerPower(double speed){
        kicker.setPower(-speed);
    }

    public void setKickerLeftPower(double speed) {
        kickerLeft.setPower(speed);
    }
    public double getOuttakeVelocity(){
        return outtakeDrive.getVelocity();
    }

}

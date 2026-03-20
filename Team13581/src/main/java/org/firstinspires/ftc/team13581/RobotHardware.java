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

package org.firstinspires.ftc.team13581;

import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


public class RobotHardware {

    private LinearOpMode myOpMode = null;

    private final double driveToTurretRatio = 60.0/26.0; // 2 rotations to 1, 60/26 teeth
    private final double turretTicksPerRevolution = driveToTurretRatio * 8192;// RevCoder CPR * ratio per 1 turret rev
    private final double turretTicksPerDegree = turretTicksPerRevolution/360;
    public static double turretTrackingOffsetDegrees = 0;
    private DcMotor frontLeftDrive  = null;
    private DcMotor backLeftDrive   = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive  = null;
    private DcMotor Intake1 = null;
    private DcMotor Intake2 = null;
    public servoDE turretHandler;
    public static int maxTurnR = 90 ;
    public static int maxTurnL = 90; // negative
    private DcMotorEx outtakeMotorR = null;
    private DcMotorEx outtakeMotorL = null;
    private CRServo turretAimR = null;
    private CRServo turretAimL = null;
    public OverflowEncoder turretE;
    private Servo hoodAim = null;

    private Servo stopper = null;

    private IMU imu = null;

    private double  headingError     = 0;
    private double  targetHeading    = 0;
    private double  axialSpeed       = 0;
    private double  lateralSpeed     = 0;
    private double  yawSpeed         = 0;
    private double  frontLeftSpeed   = 0;
    private double  backLeftSpeed    = 0;
    private double  frontRightSpeed  = 0;
    private double  backRightSpeed   = 0;
    private int     frontLeftTarget  = 0;
    private int     backLeftTarget   = 0;
    private int     frontRightTarget = 0;
    private int     backRightTarget  = 0;

    // Student Note: Camera pose (robot frame). +X forward, +Y left, +Z up (in).
    // Pitch +15° = camera looks UP 15°. Update if you remount the camera.
    // TODO(students): Measure real offsets when you rely on precise vision assists.
    private final Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, 15, 0, 0);

    private AprilTagProcessor aprilTag = null;

    //public NormalizedColorSensor colorSensor = null;

    private Integer obeliskTagId = null;
    private String obeliskMotif = null;

    private Integer goalTagId = null;
    private double goalRangeIn = Double.NaN;
    private double goalBearingDeg = Double.NaN;
    private double goalElevationDeg = Double.NaN;

    // Note: tagYawDeg is the TAG'S image rotation (not the robot's yaw). We apply this to lateral (strafe).
    private double tagYawDeg = Double.NaN;
    private static final double DESIRED_DISTANCE = 50.0; // camera-to-tag inches
    private static final double AXIAL_GAIN = 0.020; // rangeError -> axial (forward/back) speed
    private static final double LATERAL_GAIN = 0.015; // tagYawError -> lateral (strafe) speed
    private static final double YAW_GAIN = 0.010; // bearingError -> yaw (turn) speed
    public static final double MAX_AUTO_AXIAL = 0.90;
    public static final double MAX_AUTO_LATERAL = 0.90;
    public static final double MAX_AUTO_YAW = 0.70;

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

    public static final double V_Center = 0.5 ;

    static final double COUNTS_PER_MOTOR_REV  = 537.7;
    static final double DRIVE_GEAR_REDUCTION  = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 3.77953;
    static final double COUNTS_PER_INCH       = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double HEADING_THRESHOLD = 1.0;

    static final double P_AXIAL_GAIN   = 0.02;
    static final double P_LATERAL_GAIN = 0.02;
    static final double P_YAW_GAIN     = 0.03;



    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {



        //colorSensor = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "colorSensor");


        frontLeftDrive = myOpMode.hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = myOpMode.hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = myOpMode.hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = myOpMode.hardwareMap.get(DcMotor.class, "back_right_drive");

        Intake1 = myOpMode.hardwareMap.get(DcMotor.class, "intake1");
        Intake2 = myOpMode.hardwareMap.get(DcMotor.class, "intake2");

        outtakeMotorR = myOpMode.hardwareMap.get(DcMotorEx.class, "outtake_motor_r");
        outtakeMotorL = myOpMode.hardwareMap.get(DcMotorEx.class, "outtake_motor_l");

        turretAimR = myOpMode.hardwareMap.get(CRServo.class, "turret_aim_r");
        turretAimL = myOpMode.hardwareMap.get(CRServo.class, "turret_aim_l");
        turretE = new OverflowEncoder( new RawEncoder( myOpMode.hardwareMap.get(DcMotorEx.class, "intake1")));

        hoodAim = myOpMode.hardwareMap.get(Servo.class, "hood_aim");

        stopper = myOpMode.hardwareMap.get(Servo.class, "stopper");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        imu.resetYaw();

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        Intake1.setDirection(DcMotor.Direction.FORWARD);
        Intake2.setDirection(DcMotor.Direction.REVERSE);


        outtakeMotorR.setDirection(DcMotor.Direction.FORWARD);
        outtakeMotorL.setDirection(DcMotor.Direction.REVERSE);

/*
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

 */

        outtakeMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtakeMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);



        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtakeMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        turretHandler = new servoDE(turretE, turretAimR, turretAimL);

        outtakeMotorR.setVelocityPIDFCoefficients(100, 1, 3, 1);

        initAprilTag();

        while(myOpMode.opModeInInit()) {
            myOpMode.telemetry.addData("Status", "Hardware Initialized");
            myOpMode.telemetry.addData("Heading", "%4.0f", getHeading());
            myOpMode.telemetry.update();
        }
    }

    //public float[] getHSVColor(){
        //float[] hsvArray = new float[3];
        //Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsvArray);
        //return hsvArray;
    //}

    //public float[] getRGBColor(){
        //float[] rgbArray =  {colorSensor.getNormalizedColors().red, colorSensor.getNormalizedColors().green, colorSensor.getNormalizedColors().blue};
       // return rgbArray;
    //}

    public void teleOpRobotCentric(double axial, double lateral, double yaw) {

        double frontLeftPower  = (axial + lateral + yaw);
        double frontRightPower = (axial - lateral - yaw);
        double backLeftPower   = (axial - lateral + yaw);
        double backRightPower  = (axial + lateral - yaw);

        double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (max > 0.9) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        setDrivePower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }
    public double getOuttakeRVel(){
        return outtakeMotorR.getVelocity();
    }
    public void setIntake1(double p1) {
        Intake1.setPower(p1);}
    public void setIntake2(double p2) {Intake2.setPower(p2);}
    public void setBackPower(double bp) {setShootSpeed(bp*2600);}
    public double getBackPower() {return outtakeMotorR.getPower();}
    public void setShootSpeed(double v) {
        outtakeMotorR.setVelocity(v);
        outtakeMotorL.setVelocity(v);
    }

    public void setHoodPos(double pos) {
        hoodAim.setPosition(pos);}
    public double getHoodPos() { return hoodAim.getPosition();}

    public void setStopper(double pos) {
        stopper.setPosition(pos);
    }

    public void setTurretPower(double p) {
        turretAimR.setPower(p);
        turretAimL.setPower(p);

    }

    public void setTurretPosRelative(double deg){
        deg += turretHandler.getCurrentPosition()/turretTicksPerDegree;
        deg = Range.clip(deg, -maxTurnL, maxTurnR);

        turretHandler.setTargetPos((int) (deg*turretTicksPerDegree));
    }

    public void setTurretPosAbsolute(double deg){
        deg = Range.clip(deg, -maxTurnL, maxTurnR);

        turretHandler.setTargetPos((int) (deg*turretTicksPerDegree));
    }

    public double getCurrentTurretDegreePos(){
        return turretHandler.getCurrentPosition()/ turretTicksPerDegree;
    }


    public void teleOpFieldCentric(double axial, double lateral, double yaw) {

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double lateralRotation = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
        double axialRotation = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);

        double frontLeftPower  = axialRotation + lateralRotation + yaw;
        double frontRightPower = axialRotation - lateralRotation - yaw;
        double backLeftPower   = axialRotation - lateralRotation + yaw;
        double backRightPower  = axialRotation + lateralRotation - yaw;

        double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (max > 0.9) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        setDrivePower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    public void setDrivePower(double frontLeftWheel, double frontRightWheel, double backLeftWheel, double backRightWheel) {
        frontLeftDrive.setPower(frontLeftWheel);
        frontRightDrive.setPower(frontRightWheel);
        backLeftDrive.setPower(backLeftWheel);
        backRightDrive.setPower(backRightWheel);

    }

    public void autoRobotCentric(double maxAxialSpeed, double distance, double heading) {

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
            teleOpRobotCentric(axialSpeed, 0, 0);

            while (myOpMode.opModeIsActive() &&
                    (frontLeftDrive.isBusy() && backLeftDrive.isBusy() &&
                            frontRightDrive.isBusy() && backRightDrive.isBusy())) {
                yawSpeed = getSteeringCorrection(heading, P_AXIAL_GAIN);

                if (distance < 0)
                    yawSpeed *= -1.0;

                teleOpRobotCentric(axialSpeed, 0, yawSpeed);

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

            teleOpRobotCentric(0, 0, 0);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void turnToHeading(double maxYawSpeed, double heading) {

        getSteeringCorrection(heading, P_YAW_GAIN);

        // Student Note: Turn‑in‑place until heading error is small.
        // TODO(students): Tune P_YAW_GAIN for snappier or smoother turns.
        while (myOpMode.opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            yawSpeed = getSteeringCorrection(heading, P_YAW_GAIN);

            yawSpeed = Range.clip(yawSpeed, -maxYawSpeed, maxYawSpeed);

            teleOpRobotCentric(0, 0, yawSpeed);

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

        teleOpRobotCentric(0, 0, 0);
    }

    public void holdHeading(double maxYawSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // Student Note: Briefly hold heading to let the robot settle after a move.
        while (myOpMode.opModeIsActive() && (holdTimer.time() < holdTime)) {

            yawSpeed = getSteeringCorrection(heading, P_YAW_GAIN);

            yawSpeed = Range.clip(yawSpeed, -maxYawSpeed, maxYawSpeed);

            teleOpRobotCentric(0, 0, yawSpeed);

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

        teleOpRobotCentric(0, 0, 0);
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;

        headingError = -targetHeading + getHeading();

        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        return Range.clip(headingError * proportionalGain, -1.0, 1.0);
    }

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

        FtcDashboard.getInstance().startCameraStream(visionPortal, 100);
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

        teleOpRobotCentric(axial, lateral, yaw);
        return true;
    }
    private ArrayList<double[]> dataPoints = new ArrayList<>(Arrays.asList(
            // distance, rpm, hood angle
            new double[]{32, 1300,0.58},
            new double[]{38, 1375,0.575},
            new double[]{50, 1450,0.525},
            new double[]{70, 1550,0.435},
            new double[]{80, 1600,0.45},
            new double[]{120,2000,0.425}
    ));
    private final double[] velocitySlopeList = initializeSlopeList(1);
    private final double[] angleSlopeList = initializeSlopeList(2);

    private double[] initializeSlopeList(int valueIndex){
        int maxIndex = dataPoints.size();
        double[] list = new double[maxIndex];
        for (int i=0; i<maxIndex-1; i++){
            list[i] = (
                    (dataPoints.get(i+1)[valueIndex] - dataPoints.get(i)[valueIndex]) /
                            (dataPoints.get(i+1)[0] - dataPoints.get(i)[0])
            );
        }

        return list;
    }

    public double getRegressionValue(double distance, int valueIndex){
        double returnVal;
        double[] slopeList;
        if (valueIndex == 1){
            returnVal = 1300;
            slopeList = velocitySlopeList;
        } else {
            returnVal = 0.7;
            slopeList = angleSlopeList;
        }

        double velZeroIntercept = dataPoints.get(0)[valueIndex] - (slopeList[0]* dataPoints.get(0)[0]);

        if (distance <= dataPoints.get(0)[0]){
            myOpMode.telemetry.addData("Top Reference: ", dataPoints.get(0)[0] +", " + dataPoints.get(0)[valueIndex]);
            myOpMode.telemetry.addData("Slope Reference: ", slopeList[0]);
            returnVal = velZeroIntercept + (distance* slopeList[0]);
            return returnVal;
        }

        for (int i = 1; i< slopeList.length; i++){
            if ((i == slopeList.length-1 && distance >= dataPoints.get(i)[0]) ||
                    distance > dataPoints.get(i)[0] && distance <= dataPoints.get(i+1)[0]){

                myOpMode.telemetry.addData("Bottom Reference: ", dataPoints.get(i)[0] +", " + dataPoints.get(i)[valueIndex]);
                myOpMode.telemetry.addData("Slope Reference: ", slopeList[i]);
                returnVal = dataPoints.get(i)[valueIndex] + (distance- dataPoints.get(i)[0])* slopeList[i];
                return returnVal;
            }
        }

        return returnVal;
    }

    public double getFloorDistance(){
        // Math.pow(height from camera to center AprilTag, 2)
        return Math.sqrt(Math.pow(getGoalRangeIn(), 2) - Math.pow(16,2));
    }
}

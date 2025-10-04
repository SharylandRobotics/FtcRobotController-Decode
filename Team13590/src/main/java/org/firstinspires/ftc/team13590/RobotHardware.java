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

package org.firstinspires.ftc.team13590;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class RobotHardware {

    private LinearOpMode myOpMode = null;
    private DcMotor frontLeftDrive  = null;
    private DcMotor backLeftDrive   = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive  = null;

    // private DcMotorEx flywheel;
    private DcMotorEx intakeMotor;

    private IMU imu = null;

    private double  headingError     = 0;
    private double  targetHeading    = 0;
    private double  axialSpeed       = 0;
    private double  lateralSpeed     = 0;
    private double  yawSpeed         = 0;
    private int     frontLeftTarget  = 0;
    private int     backLeftTarget   = 0;
    private int     frontRightTarget = 0;
    private int     backRightTarget  = 0;

    static final double COUNTS_PER_MOTOR_REV  = 537.7;
    static final double DRIVE_GEAR_REDUCTION  = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 3.77953;
    static final double COUNTS_PER_INCH       = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double HEADING_THRESHOLD = 1.0;

    static final double P_AXIAL_GAIN   = 0.02;
    static final double P_LATERAL_GAIN = 0.02;
    static final double P_YAW_GAIN     = 0.03;

    public RobotHardware(LinearOpMode opMode) {
        myOpMode = opMode;
    }

    public void init() {

        frontLeftDrive = myOpMode.hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = myOpMode.hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = myOpMode.hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = myOpMode.hardwareMap.get(DcMotor.class, "back_right_drive");

        // flywheel = myOpMode.hardwareMap.get(DcMotorEx.class, "flywheel");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        imu.resetYaw();

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // flywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // flywheel.setTargetPosition(0);
        // flywheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while(myOpMode.opModeInInit()) {
            myOpMode.telemetry.addData("Status", "Hardware Initialized");
            myOpMode.telemetry.addData("Heading", "%4.0f", getHeading());
            myOpMode.telemetry.update();
        }
    }

    public void teleOpRobotCentric(double axial, double lateral, double yaw) {

        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

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
        if (max > 1.0) {
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

            maxAxialSpeed = Math.abs(maxAxialSpeed);
            teleOpRobotCentric(axialSpeed, 0, 0);

            while (myOpMode.opModeIsActive() &&
                    (frontLeftDrive.isBusy() && backLeftDrive.isBusy() &&
                            frontRightDrive.isBusy() && backRightDrive.isBusy())) {
                yawSpeed = getSteeringCorrection(heading, P_AXIAL_GAIN);

                if (distance < 0)
                    yawSpeed *= -1.0;

                teleOpRobotCentric(axialSpeed, lateralSpeed, yawSpeed);
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

        while (myOpMode.opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            yawSpeed = getSteeringCorrection(heading, P_YAW_GAIN);

            yawSpeed = Range.clip(yawSpeed, -maxYawSpeed, maxYawSpeed);

            teleOpRobotCentric(0,0, yawSpeed);
        }
    }

    public void holdHeading(double maxYawSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        while (myOpMode.opModeIsActive() && (holdTimer.time() < holdTime)) {
             yawSpeed = getSteeringCorrection(heading, P_YAW_GAIN);

             yawSpeed = Range.clip(yawSpeed, -maxYawSpeed, maxYawSpeed);

             teleOpRobotCentric(0,0, yawSpeed);
        }
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;

        headingError = targetHeading - getHeading();

        while (headingError > 180) headingError -= 360;
        while (headingError <= 180) headingError += 360;

        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    // public void setFlywheelVelocity(int vel ){
        // flywheel.setVelocity(vel);
    // }
}

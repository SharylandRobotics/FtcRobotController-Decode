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

package org.firstinspires.ftc.team12395;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RobotHardware {

    // We hold a reference to the active OpMode to access hardwareMap/telemetry safely
    private LinearOpMode myOpMode = null;

    // Drivetrain motors for a mecanum chassis
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    public DcMotorEx turret, shooter, spindexer;

    private final double spoolToTurretRatio = 4; // 4 rotations to 1
    private final double turretTicksPerRevolution = spoolToTurretRatio*((((1+(46./17))) * (1+(46./11))) * 28);
    private final double turretTicksPerDegree = turretTicksPerRevolution/360;
    private final double turretMaxTPS = (312./60) * turretTicksPerRevolution;
    private final int shooterMaxTPM = 2800;

    private final double spoolToSpindexerRatio = 1;
    private final double spindexerTicksPerRevolution = spoolToSpindexerRatio*((((1+(46./17))) * (1+(46./11))) * 28);
    private final double spindexerTicksPerDegree = spindexerTicksPerRevolution/360;
    private final double spindexerMaxTPS = (312./60) * spindexerTicksPerRevolution;

    // Servos
    private Servo counterFlip, hoodAngle, gate, pincher;

    private final double g = 9.81;

    // IMU is used for field-centric heading
    private IMU imu;

    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize hardware mappings and base motor/IMU configuration.
     * Call once from your OPMode before driving.
     */
    public void init() {
        // --- HARDWARE MAP NAMES ---
        frontLeftDrive = myOpMode.hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = myOpMode.hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = myOpMode.hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = myOpMode.hardwareMap.get(DcMotor.class, "back_right_drive");

        turret = myOpMode.hardwareMap.get(DcMotorEx.class, "turret");
        shooter = myOpMode.hardwareMap.get(DcMotorEx.class, "shooter");
        //spindexer = myOpMode.hardwareMap.get(DcMotorEx.class, "spindexer");


        counterFlip = myOpMode.hardwareMap.get(Servo.class, "base_left");
        hoodAngle = myOpMode.hardwareMap.get(Servo.class, "hood_angle");
        gate = myOpMode.hardwareMap.get(Servo.class, "gate");
        pincher = myOpMode.hardwareMap.get(Servo.class, "pincher");

        // --- IMU ORIENTATION ---
        // TODO(STUDENTS): Update if your Control/Expansion Hub is mounted differently.
        // The two enums MUST reflect the physical orientation of the REV Hub on the robot.
        // WHY: Field-centric depends on accurate yaw; wrong orientation => wrong heading rotations.
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,        // e.g., logo pointing up
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));  // e.g., USB ports towards right

        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        // NOTE: Reset yaw at init so heading starts ~0 at OpMode start.
        // If you prefer "press A to zero heading", move this to your OpMode and bind to a button.
        imu.resetYaw();

        // --- MOTOR DIRECTIONS ---
        // NOTE: these reversals are common for mecanum so "axial + lateral" maps correctly.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        turret.setDirection(DcMotorEx.Direction.FORWARD);
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        spindexer.setDirection(DcMotorEx.Direction.FORWARD);

        // --- ENCODER MODES ---
        // WHY: Reset once at init for a clean baseline; then RUN_USING_ENCODER for closed-loop speed control if needed.
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // NOTE: BRAKE helps with precise stopping; FLOAT cna feel smoother when coasting.
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        spindexer.setTargetPosition(0);
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // SERVO POSITIONS

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

    public void setTurretPositionAbsolute(double deg){
        turret.setTargetPosition((int) (deg*turretTicksPerDegree));

        turret.setVelocity(turretMaxTPS);

        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setTurretPositionAbsolute(double deg, double tps){
        turret.setTargetPosition((int) (deg*turretTicksPerDegree));

        turret.setVelocity(Range.clip(tps, 0, 1)*turretMaxTPS);

        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setShooterVelocity(double tPs){
        shooter.setVelocity(Range.clip(tPs, 0, shooterMaxTPM));
    }

    /**
     *
     * @return rpm in rps, not tps
     * max rpm is 100. (6000 mRPM / 60 sec = 100 mRPS * 28 TPR = 2800 mTPS)
     */
    public double getShooterVelocity(){
        return shooter.getVelocity()/28;
    }

    public void setHoodAngle(double angle){
        hoodAngle.setPosition(angle);
    }


    private int encoderToDegTurret(double deg){
        return (int) (deg* turretTicksPerDegree);
    }

    public double[] getTurretAzimuth(){
        double tDeg = turret.getCurrentPosition()/ turretTicksPerDegree;
        double deg = (tDeg) % 360;
        return new double[]{ (tDeg-deg)/360, tDeg % 360};
    }

    public double getCurrentTurretDegreePos(){
        return turret.getCurrentPosition()/ turretTicksPerDegree;
    }

    /**
     * angle solver
     * @param xT x which the curve will fall on (distance)
     * @param v given velocity
     * @return (0) the low arc, (1) the high arc)
     */
    public double[] findTurretAngles(double xT, double v){
        double yT = 38.75;
        double discriminant = Math.sqrt( Math.pow(v,4) - g*((g*xT*xT) + (2*yT*v*v)));

        return new double[] {Math.atan((v * v - discriminant) / g * xT), Math.atan((v * v + discriminant) / g * xT)};
    }

    /**
     * graph function
     * @param x distance to check
     * @param v given velocity
     * @param z given angle
     * @return distance from floor + offset the projectile will be at
     */
    public double trajectoryOf(double x, double v, double z){
        return ( x*Math.tan(z) -( (g*x*x)/(2*v*v*Math.cos(z)*Math.cos(z) ) ) );
    }

    /**
     * checks if a given velocity can reach a given coordinate
     * @param x target x
     * @param y target y
     * @param v given velocity
     * @return boolean response
     */
    public boolean shotPossible(double x, double y, double v){

        return Math.sqrt( (x*2*g)/(Math.sin(2*(Math.atan(2*y/x)))) ) == v;
    }

    public double getCurrentSpindexerDegreesPos(){
        return spindexer.getCurrentPosition()/spindexerTicksPerDegree;
    }

    public double[] getSpindexerAzimuth(){
        double tDeg = spindexer.getCurrentPosition()/ spindexerTicksPerDegree;
        double deg = (tDeg) % 360;
        return new double[]{ (tDeg-deg)/360, tDeg % 360};
    }

    public void setSpindexerRelativeAngle(double angle){
        spindexer.setTargetPosition( spindexer.getCurrentPosition() + (int) (angle*spindexerTicksPerDegree));
    }

    public void setSpindexerAbsoluteCircleAngle(double angle){
        spindexer.setTargetPosition( (int) (  (getSpindexerAzimuth()[0]*spindexerTicksPerRevolution)
                                            + (angle*spindexerTicksPerDegree) ) );
    }

    public void setSpindexerAbsoluteAngle(double revolutions, double angle){
        spindexer.setTargetPosition( (int) ( (revolutions*spindexerTicksPerRevolution) + (angle*spindexerTicksPerDegree) ) );
    }

    public void setSpindexerAbsoluteAngle(double angle){
        spindexer.setTargetPosition( (int) (angle*spindexerTicksPerDegree) );
    }
}

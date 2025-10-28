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

import android.graphics.Color;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

public class RobotHardware {

    // We hold a reference to the active OpMode to access hardwareMap/telemetry safely
    private LinearOpMode myOpMode = null;

    public Limelight3A limelight;
    public LLResult result;

    public NormalizedColorSensor colorSensor;

    public static String mag = "GPP";
    public static String pattern = "000";
    public int chamber = 0;

    // Drivetrain motors for a mecanum chassis
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private final double wheelDiameterInches = 3.77953;
    private final double wheelsTicksPerRev = ((((1+(46./17))) * (1+(46./11))) * 28);
    private final double wheelsTicksPerInch = wheelsTicksPerRev/(wheelDiameterInches*Math.PI);



    public DcMotorEx turret, shooter, spindexer, intake;

    public static int maxTurnR = 100 ;
    public static int maxTurnL = 100; // negative

    private final double spoolToTurretRatio = 4; // 4 rotations to 1
    private final double turretTicksPerRevolution = spoolToTurretRatio*((((1+(46./17))) * (1+(46./11))) * 28);
    private final double turretTicksPerDegree = turretTicksPerRevolution/360;
    private final double turretMaxTPS = (312./60) * turretTicksPerRevolution;
    private final int shooterMaxTPM = 2800;

    private final static double spoolToSpindexerRatio = 1;
    private final static double spindexerTicksPerRevolution = spoolToSpindexerRatio*((((1+(46./17))) * (1+(46./11))) * 28);
    public final static double spindexerTicksPerDegree = spindexerTicksPerRevolution/360;
    private final double spindexerMaxTPS = (312./60) * spindexerTicksPerRevolution;

    private int spindexerTarget = 0;

    // Servos
    private Servo xArm, hoodAngle, hoodAngle2;

    // physics

    private final double g = 9.81;
    private final double xOffset = 5.5; //cm
    private final double yOffset = 16.5*2.54; //cm
    private final double verticalTargetDistance = 70-yOffset;


    // IMU is used for field-centric heading
    private IMU imu;

    // lemoine stuff

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
    public static final double AXIAL_SPEED = 0.6;
    public static final double LATERAL_SPEED = 0.4;
    public static final double YAW_SPEED = 0.2;
    static final double HEADING_THRESHOLD = 0.25;

    static final double P_AXIAL_GAIN = 0.03;
    static final double P_LATERAL_GAIN = 0.03;
    static final double P_YAW_GAIN = 0.02;

    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize hardware mappings and base motor/IMU configuration.
     * Call once from your OPMode before driving.
     */
    public void init() {
        // --- HARDWARE MAP NAMES ---
        limelight = myOpMode.hardwareMap.get(Limelight3A.class, "limelight-rfc");
        limelight.pipelineSwitch(1);

        frontLeftDrive = myOpMode.hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = myOpMode.hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = myOpMode.hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = myOpMode.hardwareMap.get(DcMotor.class, "back_right_drive");

        turret = myOpMode.hardwareMap.get(DcMotorEx.class, "turret");
        shooter = myOpMode.hardwareMap.get(DcMotorEx.class, "shooter");
        spindexer = myOpMode.hardwareMap.get(DcMotorEx.class, "spindexer");
        intake = myOpMode.hardwareMap.get(DcMotorEx.class, "intake");

        colorSensor = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "color_sensor");

        xArm = myOpMode.hardwareMap.get(Servo.class, "xArm");
        hoodAngle = myOpMode.hardwareMap.get(Servo.class, "hood_angle");
        hoodAngle2 = myOpMode.hardwareMap.get(Servo.class, "hood_angle2");

        // --- IMU ORIENTATION ---
        // TODO(STUDENTS): Update if your Control/Expansion Hub is mounted differently.
        // The two enums MUST reflect the physical orientation of the REV Hub on the robot.
        // WHY: Field-centric depends on accurate yaw; wrong orientation => wrong heading rotations.
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,        // e.g., logo pointing up
                RevHubOrientationOnRobot.UsbFacingDirection.UP));  // e.g., USB ports towards right

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
        intake.setDirection(DcMotorEx.Direction.FORWARD);

        // --- ENCODER MODES ---
        // WHY: Reset once at init for a clean baseline; then RUN_USING_ENCODER for closed-loop speed control if needed.
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // NOTE: BRAKE helps with precise stopping; FLOAT cna feel smoother when coasting.
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


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

        spindexer.setVelocityPIDFCoefficients(14,4,1,4);
        spindexer.setPower(0.3);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(100, 3, 3, 0);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // SERVO POSITIONS

        hoodAngle.setPosition(1);

        xArm.setPosition(1);

        myOpMode.telemetry.addData("Status", "Hardware Initialized");
        myOpMode.telemetry.addData("PIDF", shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
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

    public void resetDriveEncoder(){
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void driveToEncoderRobotCentric(double distance, double directionAngle, double power){
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        directionAngle = Math.toRadians(directionAngle);

        double lateralTicks = directionAngle/Math.abs(directionAngle)*
                distance*Math.cos((Math.abs(directionAngle)));

        double axialTicks = directionAngle/Math.abs(directionAngle)*
                distance*Math.sin((Math.abs(directionAngle)));


        // Input rotation for field frame
        double lateralRotation =  (Math.cos(directionAngle) -  Math.sin(directionAngle));
        double axialRotation =  (Math.sin(directionAngle) +  Math.cos(directionAngle));

        // WHY: Standard mecanum mixing (A + L + Y. etc.). Values may exceed |1|; we normalize below.
        double frontLeftPower  = axialRotation + lateralRotation ;
        double frontRightPower = axialRotation - lateralRotation ;
        double backLeftPower   = axialRotation - lateralRotation ;
        double backRightPower  = axialRotation + lateralRotation ;

        // Normalize so that the highest magnitude is 1.0, preserving ratios
        double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

        max *= Range.clip(power,0,1);
        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }



        setDrivePower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    public void driveEncoder(double speed, double leftFrontInches, double leftBackInches, double rightFrontInches, double rightBackInches){
        // drives only while myOpMode is active
        if(myOpMode.opModeIsActive()){


            //determine new target position
            int leftFrontTarget = frontLeftDrive.getCurrentPosition() + (int)(leftFrontInches * wheelsTicksPerInch);
            int leftBackTarget = backLeftDrive.getCurrentPosition() + (int)(leftBackInches * wheelsTicksPerInch);
            int rightFrontTarget = frontRightDrive.getCurrentPosition() + (int)(rightFrontInches * wheelsTicksPerInch);
            int rightBackTarget = backRightDrive.getCurrentPosition() + (int)(rightBackInches * wheelsTicksPerInch);

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


    public void setIntakeSpeed(int vel){
        intake.setVelocity(vel);
    }

    public void setTurretPositionAbsolute(double deg){
        deg = Range.clip(deg, -maxTurnL, maxTurnR);
        turret.setTargetPosition((int) (deg*turretTicksPerDegree));

        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turret.setVelocity(turretMaxTPS);
    }

    public void setTurretPositionAbsolute(double deg, double tps){
        deg = Range.clip(deg, -maxTurnL, maxTurnR);
        turret.setTargetPosition((int) (deg*turretTicksPerDegree));

        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turret.setVelocity(Range.clip(tps, 0, 1)*turretMaxTPS);
    }

    public void setTurretPositionRelative(double deg, double tps){
        deg += getCurrentTurretDegreePos();
        deg = Range.clip(deg, -maxTurnL, maxTurnR);
        turret.setTargetPosition((int) deg);

        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turret.setVelocity(Range.clip(tps, 0, 1)*turretMaxTPS);
    }

    public void setTurretPositionRelative(double deg){
        deg += getCurrentTurretDegreePos();
        deg = Range.clip(deg, -maxTurnL, maxTurnR);

        turret.setTargetPosition((int) (deg*turretTicksPerDegree));

        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turret.setVelocity(turretMaxTPS);
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
        hoodAngle2.setPosition(1-angle);

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

    public void spindexerHandler(int targetAdd){
        spindexer.setTargetPosition( (int) ( (spindexerTarget + targetAdd) * spindexerTicksPerDegree) );

        spindexerTarget += targetAdd;

        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        spindexer.setVelocity(800);
    }

    public void setSpindexerRelativeAngle(double angle){
        spindexer.setTargetPosition( spindexer.getCurrentPosition() + (int) (angle*spindexerTicksPerDegree));

        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void turnSpindexerRight(double angle){

        spindexer.setTargetPosition( spindexer.getCurrentPosition() + (int) (-angle*spindexerTicksPerDegree));

        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void setArmPos(double pos){
        xArm.setPosition(Range.clip(pos, 0, 1));
    }

    private double[] getValuesToTarget(){
        double tx = limelight.getLatestResult().getTx();
        double ty = limelight.getLatestResult().getTy();
        Position pose = limelight.getLatestResult().getBotpose_MT2().getPosition();

        double yDistance = verticalTargetDistance /Math.tan(ty);
        double xDistance = Math.tan(tx)*yDistance;

        double groundDistance = yDistance/Math.cos(tx);

        return new double[]{ xDistance, yDistance, groundDistance};
    }

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

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public boolean processLLresult(){
        result = limelight.getLatestResult();
        return result != null && result.isValid();
    }

    public double homeToAprilTag(){
        if (processLLresult()){

            List<LLResultTypes.FiducialResult> fresult = result.getFiducialResults();

            myOpMode.telemetry.addData("Closest Tag ID: ", fresult.get(0).getFiducialId());
            myOpMode.telemetry.addData("Tags: ", fresult.size());


            for (LLResultTypes.FiducialResult fiducial : fresult){
                if (fiducial.getFiducialId() == 21 || fiducial.getFiducialId() == 22 || fiducial.getFiducialId() == 23 || fiducial.getFiducialId() == 24){
                    fresult.remove(fiducial);
                }
            }

            if (!fresult.isEmpty()) {


                double tx = Math.round(fresult.get(0).getTargetXDegrees()*100)/100.;

                myOpMode.telemetry.addData("turning deg: ", tx);
                return tx;

            }

        }
        return Double.NaN;
    }

    public void processObelisk(){
        if (processLLresult()){
            List<LLResultTypes.FiducialResult> fresult = result.getFiducialResults();

            myOpMode.telemetry.addData("Closest Tag ID: ", fresult.get(0).getFiducialId());
            myOpMode.telemetry.addData("Tags: ", fresult.size());


            for (LLResultTypes.FiducialResult fiducial : fresult){
                if (fiducial.getFiducialId() == 20 || fiducial.getFiducialId() == 24){
                    fresult.remove(fiducial);
                }
            }

            if (!fresult.isEmpty()) {
                int closestObelisk = fresult.get(0).getFiducialId();
                if (closestObelisk == 21){
                    pattern = "GPP";
                    myOpMode.telemetry.addData("Tag 21: ", "GPP");
                } else if (closestObelisk == 22) {
                    pattern = "PGP";
                    myOpMode.telemetry.addData("Tag 22: ", "PGP");
                } else if (closestObelisk == 23){
                    pattern = "PPG";
                    myOpMode.telemetry.addData("Tag 23: ", "PPG");
                }
            }
        }
    }

    public int[] solvePattern(){
        if (!mag.contains("0") && mag.contains("G") && !pattern.contains("0")){
            // if I have a full mag with Green and know the pattern
            int greenIndex = mag.indexOf("G");
            if (pattern.equals("GPP")){
                if (greenIndex == chamber){// if green is selected
                    return new int[] {0, 2};// don't move, turn right twice
                } else if (Character.toString( mag.charAt((chamber + 1) % mag.length()) ) != "G" ){
                    // if the color to my right isn't green, turn left, then turn right twice
                    return new int[] {-1, 2};
                } else {
                    // the color to my right is green, turn right, then turn right twice
                    return new int[] {1, 2};
                }

            } else if (pattern.equals("PGP")){
                if (greenIndex == chamber){ // if green is selected
                    return new int[] {-1, 2}; //  turn left, then turn right twice
                } else if (Character.toString( mag.charAt((chamber + 1) % mag.length()) ) != "G" ){
                    // if the color to my right isn't green, turn right, then turn right twice
                    return new int[] {1, 2};
                } else {
                    // the color to my right is green, don't move, turn right twice
                    return new int[] {0, 2};
                }

            } else if (pattern.equals("PPG")){
                if (greenIndex == chamber){ // if green is selected
                    return new int[] {1, 2}; //  turn right, then turn right twice
                } else if (Character.toString( mag.charAt((chamber + 1) % mag.length()) ) != "G" ){
                    // if the color to my right isn't green, don't move, turn right twice
                    return new int[] {0, 2};
                } else {
                    // the color to my right is green, turn left, then turn right twice
                    return new int[] {-1, 2};
                }
            }
        }
        return null;
    }

    public int[] solveAltPattern(String setPattern){
        String reconstruct = String.valueOf(mag.charAt(chamber) + mag.charAt((chamber + 1) % 3) + mag.charAt((chamber + 2) % 3));
        if  (reconstruct.equals(setPattern)) {
            return new int[] {0, 2};
        }
        return null;
    }
}

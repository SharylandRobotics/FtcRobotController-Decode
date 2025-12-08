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

import android.content.Context;
import android.graphics.Color;
import android.media.AudioManager;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

@Config
public class RobotHardware {

    // We hold a reference to the active OpMode to access hardwareMap/telemetry safely
    private LinearOpMode myOpMode = null;

    public Limelight3A limelight;
    public LLResult result;

    public NormalizedColorSensor colorSensor;
    public static colorTypes scannedColor = colorTypes.UNKNOWN;

    public String mag = "GPP"; // EACH +1 ON THE MAG INDEX IS ONE CW TURN
    public String pattern = "PPG";// a pattern is better than no pattern
    public static int chamber = 0;

    // Drivetrain motors for a mecanum chassis
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

    public DcMotorEx shooter2, shooter, spindexer, intake;
    public CRServo turretR, turretL;
    public OverflowEncoder turretE, spindexerE;

    public servoDrivenEncoder turretHandler;

    public static int maxTurnR = 100 ;
    public static int maxTurnL = 100; // negative

    private final double driveToTurretRatio = 3.2; // 3.2 rotations to 1, 80/25 teeth
    private final double turretTicksPerRevolution = driveToTurretRatio *8192;// RevCoder CPR * ratio per 1 turret rev
    public final double turretTicksPerDegree = turretTicksPerRevolution/360;
    private final double turretMaxTPS = (312./60) * turretTicksPerRevolution;
    private final int shooterMaxTPM = 2800;

    private final static double spoolToSpindexerRatio = 1;
    private final static double spindexerTicksPerRevolution = spoolToSpindexerRatio*((((1+(46./17))) * (1+(46./11))) * 28);
    public final static double spindexerTicksPerDegree = spindexerTicksPerRevolution/360;
    public final double spindexerETicksPerRevolution = 8192;
    public final double spindexerETicksPerDegree = spindexerETicksPerRevolution/360;
    private final double spindexerMaxTPS = (312./60) * spindexerTicksPerRevolution;

    public double spindexerFudge = 0;
    public int spindexerTarget = 0;

    // Servos
    private Servo hoodAngle;

    // physics

    private final double g = 9.81;
    private final double xOffset = 5.5; //cm
    private final double yOffset = 16.5*2.54; //cm
    private final double verticalTargetDistance = 70-yOffset;


    // IMU is used for field-centric heading
    private IMU imu;

    // COLOR SENSOR

    public int[] prevColor = null;
    public int[] currentColor;

    boolean orbPreload;
    boolean orbDeepPreload;
    private Context appContext;
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
        //appContext = opmode.hardwareMap.appContext;
    }

    /**
     * Initialize hardware mappings and base motor/IMU configuration.
     * Call once from your OPMode before driving.
     */
    public void init() {
        appContext = myOpMode.hardwareMap.appContext;
        // --- HARDWARE MAP NAMES ---
        limelight = myOpMode.hardwareMap.get(Limelight3A.class, "limelight-rfc");

        frontLeftDrive = myOpMode.hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = myOpMode.hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = myOpMode.hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = myOpMode.hardwareMap.get(DcMotor.class, "back_right_drive");

        shooter2 = myOpMode.hardwareMap.get(DcMotorEx.class, "turret");

        turretE = new OverflowEncoder( new RawEncoder( myOpMode.hardwareMap.get(DcMotorEx.class, "turret")));
        turretR = myOpMode.hardwareMap.get(CRServo.class, "turretR");
        turretL = myOpMode.hardwareMap.get(CRServo.class, "turretL");
        hoodAngle = myOpMode.hardwareMap.get(Servo.class, "hood_angle");


        shooter = myOpMode.hardwareMap.get(DcMotorEx.class, "shooter");
        spindexer = myOpMode.hardwareMap.get(DcMotorEx.class, "spindexer");
        spindexerE = new OverflowEncoder(new RawEncoder( myOpMode.hardwareMap.get(DcMotorEx.class, "intake")));

        intake = myOpMode.hardwareMap.get(DcMotorEx.class, "intake");



        colorSensor = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "color_sensor");

        // --- IMU ORIENTATION ---
        // TODO: UPDATE ALONGSIDE ROADRUNNER
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

        shooter2.setDirection(DcMotorEx.Direction.FORWARD);
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        spindexer.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setDirection(DcMotorEx.Direction.REVERSE);

        turretR.setDirection(DcMotorSimple.Direction.REVERSE);
        turretL.setDirection(DcMotorSimple.Direction.REVERSE);
        turretE.setDirection(DcMotorSimple.Direction.REVERSE);

        spindexerE.setDirection(DcMotorSimple.Direction.REVERSE);

        turretHandler = new servoDrivenEncoder(turretE, turretR, turretL);

        // --- ENCODER MODES ---
        // WHY: Reset once at init for a clean baseline; then RUN_USING_ENCODER for closed-loop speed control if needed.
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // NOTE: BRAKE helps with precise stopping; FLOAT cna feel smoother when coasting.
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        spindexer.setTargetPosition(0);
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        spindexer.setVelocityPIDFCoefficients(14,4,1,4);
        spindexer.setPower(0.3);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(100, 5, 1, 5);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        colorSensor.setGain(100);

        // SERVO POSITIONS

        hoodAngle.setPosition(0.3);

        limelight.start();
        limelight.pipelineSwitch(1);

        //pattern = "PPG";
        mag = "GPP";

        SoundPlayer.getInstance().setMasterVolume(1.0f);
        AudioManager am = (AudioManager) appContext.getSystemService(Context.AUDIO_SERVICE);
        int max = am.getStreamMaxVolume(AudioManager.STREAM_MUSIC);
        am.setStreamVolume(AudioManager.STREAM_MUSIC, max, 0);

        myOpMode.telemetry.addData("Status", "Hardware Initialized");
        myOpMode.telemetry.addData("Sound Preloaded: ", SoundPlayer.getInstance().preload(appContext, R.raw.orb));
        myOpMode.telemetry.addData("Sound2 Preloaded: ", SoundPlayer.getInstance().preload(appContext, R.raw.orb_deep));
        myOpMode.telemetry.addData("Sound3 Preloaded: ", SoundPlayer.getInstance().preload(appContext, R.raw.anvil_break));
        //myOpMode.telemetry.addData("PIDF", shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        myOpMode.telemetry.update();

    }


    public void playBeep(String file) {
        if (Objects.equals(file, "orb")) {
            SoundPlayer.getInstance().startPlaying(appContext, R.raw.orb);
            SoundPlayer.getInstance().preload(appContext, R.raw.orb);
        } else if (Objects.equals(file, "orbDeep")){
            SoundPlayer.getInstance().startPlaying(appContext, R.raw.orb_deep);
            SoundPlayer.getInstance().preload(appContext, R.raw.orb_deep);
        } else if (Objects.equals(file, "break")){
            SoundPlayer.getInstance().startPlaying(appContext, R.raw.anvil_break);
            SoundPlayer.getInstance().preload(appContext, R.raw.anvil_break);
        }

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

    public void disableDriveEncoders(){
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void setIntakeSpeed(double vel){
        intake.setPower(vel);
    }

    public void setTurretHandlerRelative(double deg){
        deg += turretHandler.getCurrentPosition()/turretTicksPerDegree;
        deg = Range.clip(deg, -maxTurnL, maxTurnR);

        turretHandler.setTargetPos((int) (deg*turretTicksPerDegree));
    }

    public void setTurretHandlerAbsolute(double deg){
        deg = Range.clip(deg, -maxTurnL, maxTurnR);

        turretHandler.setTargetPos((int) (deg*turretTicksPerDegree));
    }

    public void setShooterVelocity(double tPs){
        shooter.setVelocity(Range.clip(tPs, 0, shooterMaxTPM));
        shooter2.setPower(shooter.getPower());
    }

    /**
     *
     * @return rpm in rps, not tps
     * max rpm is 100. (6000 mRPM / 60 sec = 100 mRPS * 28 TPR = 2800 mTPS)
     */

    public void setHoodAngle(double angle){
        hoodAngle.setPosition(angle);
    }

    public double getCurrentTurretDegreePos(){
        return turretHandler.getCurrentPosition()/ turretTicksPerDegree;
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

    // 8192
    public void getSpindexerOffset(){
        spindexerFudge = (spindexerE.getPositionAndVelocity().position/spindexerETicksPerDegree) - (spindexer.getCurrentPosition()/spindexerTicksPerDegree);
    }

    public void spindexerHandler(int targetAdd){
        getSpindexerOffset();
        spindexer.setTargetPosition( (int) ( (spindexerTarget + targetAdd - spindexerFudge) * spindexerTicksPerDegree) );

        spindexerTarget += targetAdd;

        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        spindexer.setVelocity(900);
        //  -intake-
        //    (0)
        // (1)   (2)
        int turns = targetAdd / 120;

        if (turns < 0){// cw
            chamber = (chamber + Math.abs(turns)) % 3;
        } else if (turns > 0){// ccw
            chamber = (chamber + 2*Math.abs(turns)) % 3;
        }
    }

    public void maintainSpindexerHandler(){
        spindexer.setTargetPosition( (int) ( (spindexerTarget + spindexerFudge) * spindexerTicksPerDegree) );

        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        spindexer.setVelocity(800);
    }

    public void spindexerHandler(int targetAdd,  int vel){
        getSpindexerOffset();
        spindexer.setTargetPosition( (int) ( (spindexerTarget + targetAdd - spindexerFudge) * spindexerTicksPerDegree) );

        spindexerTarget += targetAdd;

        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        spindexer.setVelocity(vel);
        //  -intake-
        //    (0)
        // (1)   (2)
        int turns = targetAdd / 120;

        if (turns < 0){// cw
            chamber = (chamber + Math.abs(turns)) % 3;
        } else if (turns > 0){// ccw
            chamber = (chamber + 2*Math.abs(turns)) % 3;
        }
    }

    public void setSpindexerRelativeAngle(double angle){
        spindexer.setTargetPosition( spindexer.getCurrentPosition() + (int) (angle*spindexerTicksPerDegree));

        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public boolean processLLresult(){
        result = limelight.getLatestResult();
        return ((result != null && result.isValid()) &&
                (result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()));
    }

    public double[] homeToAprilTagBlue(){
        boolean check = processLLresult();
        if (check){

            List<LLResultTypes.FiducialResult> fresult = result.getFiducialResults();
            List<LLResultTypes.FiducialResult> fresultCC = new ArrayList<>(fresult);

            myOpMode.telemetry.addData("Closest Tag ID: ", fresult.get(0).getFiducialId());
            myOpMode.telemetry.addData("Tags: ", fresult.size());


            for (LLResultTypes.FiducialResult fiducial : fresultCC){
                if (fiducial.getFiducialId() == 21 || fiducial.getFiducialId() == 22 || fiducial.getFiducialId() == 23 || fiducial.getFiducialId() == 24){
                    fresult.remove(fiducial);
                }
            }

            if (!fresult.isEmpty()) {


                double tx = Math.round(fresult.get(0).getTargetXDegrees()*100)/100.;
                double skew  = fresult.get(0).getCameraPoseTargetSpace().getOrientation().getYaw();

                myOpMode.telemetry.addData("turning deg: ", tx);
                myOpMode.telemetry.addData("skew deg: ", skew);
                return new double[]{tx, skew};

            }

        }
        return new double[]{Double.NaN, Double.NaN};
    }

    public double[] homeToAprilTagRed(){
        boolean check = processLLresult();
        if (check){

            List<LLResultTypes.FiducialResult> fresult = result.getFiducialResults();
            List<LLResultTypes.FiducialResult> fresultCC = new ArrayList<>(fresult);

            myOpMode.telemetry.addData("Closest Tag ID: ", fresult.get(0).getFiducialId());
            myOpMode.telemetry.addData("Tags: ", fresult.size());


            for (LLResultTypes.FiducialResult fiducial : fresultCC){
                if (fiducial.getFiducialId() == 21 || fiducial.getFiducialId() == 22 || fiducial.getFiducialId() == 23 || fiducial.getFiducialId() == 20){
                    fresult.remove(fiducial);
                }
            }

            if (!fresult.isEmpty()) {


                double tx = Math.round(fresult.get(0).getTargetXDegrees()*100)/100.;
                double skew  = fresult.get(0).getCameraPoseTargetSpace().getOrientation().getYaw();

                myOpMode.telemetry.addData("turning deg: ", tx);
                myOpMode.telemetry.addData("skew deg: ", skew);
                return new double[]{tx, skew};

            }

        }
        return new double[]{Double.NaN, Double.NaN};
    }

    public boolean processObelisk(){
        if (processLLresult()){
            List<LLResultTypes.FiducialResult> fresult = result.getFiducialResults();
            List<LLResultTypes.FiducialResult> fresultCC = new ArrayList<>(fresult);

            myOpMode.telemetry.addData("Closest Tag ID: ", fresult.get(0).getFiducialId());
            myOpMode.telemetry.addData("Tags: ", fresult.size());


            for (LLResultTypes.FiducialResult fiducial : fresultCC){
                if (fiducial.getFiducialId() == 20 || fiducial.getFiducialId() == 24){
                    fresult.remove(fiducial);
                }
            }

            if (!fresult.isEmpty()) {
                int closestObelisk = fresult.get(0).getFiducialId();
                if (closestObelisk == 21){
                    pattern = "GPP";
                    myOpMode.telemetry.addData("Tag 21: ", "GPP");
                    myOpMode.telemetry.update();
                    return true;
                } else if (closestObelisk == 22) {
                    pattern = "PGP";
                    myOpMode.telemetry.addData("Tag 22: ", "PGP");
                    myOpMode.telemetry.update();

                    return true;
                } else if (closestObelisk == 23){
                    pattern = "PPG";
                    myOpMode.telemetry.addData("Tag 23: ", "PPG");
                    myOpMode.telemetry.update();
                    return true;
                }
            }
        }
        return false;
    }

    public int[] solvePattern(){
        if (!mag.contains("0") && mag.contains("G") ){
            // if I have a full mag with Green and know the pattern
            int greenIndex = mag.indexOf("G");
            switch (pattern) {
                case "GPP":
                    if (greenIndex == chamber) {// if green is selected
                        return new int[]{0, -2};// don't move, turn right twice
                    } else if (mag.charAt((chamber + 1) % mag.length()) != 'G') {
                        // if the color to my ccw isn't green, turn left, then turn right twice
                        return new int[]{1, -2};
                    } else {
                        // the color to my ccw is green, turn right (2x left), then turn right twice
                        return new int[]{2, -2};
                    }

                case "PGP":
                    if (greenIndex == chamber) { // if green is selected
                        return new int[]{1, -2}; //  turn left, then turn right twice
                    } else if (mag.charAt((chamber + 1) % mag.length()) != 'G') {
                        // if the color to my ccw isn't green, turn right (2x left), then turn right twice
                        return new int[]{2, -2};
                    } else {
                        // the color to my ccw is green, don't move, turn right twice
                        return new int[]{0, -2};
                    }

                case "PPG":
                    if (greenIndex == chamber) { // if green is selected
                        return new int[]{2, 2}; //  turn cw (2x ccw), then turn cw twice
                    } else if (mag.charAt((chamber + 1) % mag.length()) != 'G') {
                        // if the color to my ccw isn't green, don't move, turn cw twice
                        return new int[]{0, -2};
                    } else {
                        // the color to my ccw is green, turn ccw, then turn cw twice
                        return new int[]{1, -2};
                    }
            }
        }
        myOpMode.telemetry.addData(":", mag, pattern, chamber);
        return null;
    }

    public int[] solveAltPattern(String setPattern){
        String reconstruct = String.valueOf(mag.charAt(chamber) + mag.charAt((chamber + 1) % 3) + mag.charAt((chamber + 2) % 3));
        if  (reconstruct.equals(setPattern)) {
            return new int[] {0, 2};
        }
        return null;
    }

    public void setMagManualBulk(String set){
        //      (chamber on 0)         012
        // set things in a cw manner ( XYZ )
        //    0(X)
        // 1(Z)   2(Y)
        StringBuilder magBuilder = new StringBuilder(mag);
        for (int i=0; i<3; i++) {
            magBuilder.setCharAt((chamber + i) % 3, set.charAt(i));
            mag = magBuilder.toString();
        }
    }
    public void setChamberManual(Character c){
        StringBuilder magBuilder = new StringBuilder(mag);
        magBuilder.setCharAt(chamber, c);
        mag = magBuilder.toString();
    }

    public enum colorTypes {
        UNKNOWN,
        NONE,
        PURPLE,
        GREEN
    }

    public void scanColor(){
        NormalizedRGBA color = colorSensor.getNormalizedColors();
        float[] hsvValues = new float[3];
        Color.colorToHSV(color.toColor(), hsvValues);

        scannedColor = colorTypes.UNKNOWN;
        if (hsvValues[2] < 0.15){
            scannedColor = colorTypes.NONE;
        } else if (hsvValues[0] > 200 && hsvValues[0] <= 240){
            scannedColor = colorTypes.PURPLE;
        } else if (hsvValues[1] > 0.6 && hsvValues[1] < 0.75){
            scannedColor = colorTypes.GREEN;
        }
    }

    public String getMagPicture(){
        return "     ("+mag.charAt(chamber)+")  " + "\n ("+mag.charAt( (chamber+1) % 3)+")    ("+mag.charAt( (chamber+2) % 3)+")  ";
    }

    private ArrayList<double[]> velDataPoints = new ArrayList<>(Arrays.asList(
            new double[]{0, 0},
            new double[]{1, 1},
            new double[]{2, 2}
    ));

    public double getCalculatedVelocity(double distance){
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
                    return velZeroIntercept + distance*slopeList[0];
                }
            } else if (distance > velDataPoints.get(i)[0] && distance <= velDataPoints.get(i+1)[0]){
                return velDataPoints.get(i)[1] + distance*slopeList[i];
            }
        }

        return 500;
    }

    public class RoadRunnerActions {
        public class setSpindexerTarget implements Action{
            private int vel;
            private int target;
            public setSpindexerTarget(int target, int vel){
                this.vel = vel;
                this.target = target;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                spindexerHandler(target, vel);
                return false;
            }
        }
        public class scanMotif implements Action {
            public scanMotif(){

            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                return !processObelisk();
            }
        }
        public class setTurretPosition implements Action {
            private double pos;
            public setTurretPosition(double pos){
                this.pos = pos;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                setTurretHandlerAbsolute(pos);
                return !turretHandler.runToTarget();
            }
        }
        public class setShooterVelocity implements Action {
            private int vel;

            public setShooterVelocity(int vel){
                this.vel = vel;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                setShooterVelocity(vel);
                return false;
            }
        }

        public class automaticIntakeBalls implements Action{
            public automaticIntakeBalls(){

            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                // run until spindexer is full
                scanColor();
                boolean rerun = true;
                if (!spindexer.isBusy() && mag.contains("0")){
                    if (scannedColor.equals(colorTypes.PURPLE)){
                        if (mag.charAt(chamber) == '0') {
                            setChamberManual('P');
                            spindexerHandler(120);
                        }
                    } else if (scannedColor.equals(colorTypes.GREEN)){
                        if (mag.charAt(chamber) == '0') {
                            setChamberManual('G');
                            spindexerHandler(120);
                        }
                    }
                } else if (!mag.contains("0")){
                    rerun = false;
                }
                return rerun;
            }
        }

        public class shootBalls implements Action{
            public shootBalls(){

            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                spindexerHandler(-480, 600);
                return false;
            }
        }

        public class setTurretPowerZero implements Action{
            private double deg;

            public setTurretPowerZero(){

            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                turretHandler.stopServos();
                return false; // you are not done?
            }
        }

        public class sortSpindexer implements Action{

            public sortSpindexer(){

            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                spindexerHandler(120*solvePattern()[0]);
                return false;
            }
        }

        public class setHoodAngle implements Action{
            private double pos;

            public setHoodAngle(double pos){
                this.pos = pos;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                setHoodAngle(pos);
                return false;
            }
        }

        public class setIntakeVelocity implements Action{
            private double vel;
            public setIntakeVelocity(double vel){
                this.vel = vel;
            }

            @Override
            public boolean run(TelemetryPacket packet){
                setIntakeSpeed(vel);
                return false;
            }
        }

        public Action shootAllBalls(){
            return new shootBalls();
        }

        public Action automaticallyIntakeBalls(){
            return new automaticIntakeBalls();
        }

        public Action setIntakeVel(double vel){
            return new setIntakeVelocity(vel);
        }

        public Action setHoodAng(double angle){
            return new setHoodAngle(angle);
        }

        public Action sortCurrentSpindexer(){
            return new sortSpindexer();
        }

        public Action setShooterVel(int vel){
            return new setShooterVelocity(vel);
        }

        public Action stopTurretPower(){
            return new setTurretPowerZero();
        }
        public Action setTurretPos(double deg){
            return new setTurretPosition(deg);
        }
        public Action scanMotif(){
            return new scanMotif();
        }

        public Action spindexerTargetAddVel(int target, int vel){
            return new setSpindexerTarget(target, vel);
        }
    }

}

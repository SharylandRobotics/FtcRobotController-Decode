package org.firstinspires.ftc.team00000;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
public class RobotHardware {

    public static double P_AXIAL_GAIN = 0.02;
    public static double P_LATERAL_GAIN = 0.02;
    public static double P_YAW_GAIN = 0.02;
    public static double HEADING_THRESHOLD = 1.0;

    public static double DEADBAND = 0.05;
    public static double SLEW_MAX_AXIAL = 0.06;
    public static double SLEW_MAX_LATERAL = 0.06;
    public static double SLEW_MAX_YAW = 0.05;

    public static final double COUNTS_PER_MOTOR_REV = 537.7;
    public static final double DRIVE_GEAR_REDUCTION = 1.0;
    public static final double WHEEL_DIAMETER_INCHES = 3.78;
    public static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    private final LinearOpMode opMode;

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private IMU imu;

    private final HeadingSubsystem heading = new HeadingSubsystem();
    private final DriveSubsystem drive = new DriveSubsystem();

    private final ElapsedTime runtime = new ElapsedTime();

    public RobotHardware(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init() {
        frontLeftDrive = opMode.hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = opMode.hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = opMode.hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = opMode.hardwareMap.get(DcMotor.class, "backRightDrive");

        frontLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        imu = opMode.hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );
        imu.initialize(parameters);

        heading.init();
        drive.init();
        runtime.reset();
    }

    public void start() {
        heading.reset();
        runtime.reset();
    }

    public void periodic() {
        heading.update();
        drive.update();
    }

    public void stop() {
        drive.stopAll();
    }

    public void teleOpRobotCentric(double axial, double lateral, double yaw) {
        drive.driveRobotCentric(applyDeadband(axial), applyDeadband(lateral), applyDeadband(yaw));
    }

    public void teleOpFieldCentric(double axial, double lateral, double yaw) {
        drive.driveFieldCentric(applyDeadband(axial), applyDeadband(lateral), applyDeadband(yaw), getHeading());
    }

    public void autoRobotCentric(double inches, double maxPower, double headingDeg) {
        int move = (int) Math.round(inches * COUNTS_PER_INCH);

        setRunToPosition(frontLeftDrive,  frontLeftDrive.getCurrentPosition()  + move);
        setRunToPosition(frontRightDrive, frontRightDrive.getCurrentPosition() + move);
        setRunToPosition(backLeftDrive,   backLeftDrive.getCurrentPosition()   + move);
        setRunToPosition(backRightDrive,  backRightDrive.getCurrentPosition()  + move);

        setAllPowers(maxPower);

        while (opMode.opModeIsActive()
                && (frontLeftDrive.isBusy() || frontRightDrive.isBusy() || backLeftDrive.isBusy() || backRightDrive.isBusy())) {

            double yawSpeed = getSteeringCorrection(headingDeg, P_YAW_GAIN);
            double left = Range.clip(maxPower - yawSpeed, -1, 1);
            double right = Range.clip(maxPower + yawSpeed, -1, 1);
            frontLeftDrive.setPower(left);
            backLeftDrive.setPower(left);
            frontRightDrive.setPower(right);
            backRightDrive.setPower(right);

            opMode.idle();
        }

        drive.stopAll();
        for (DcMotor m : new DcMotor[]{frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void turnToHeading(double targetHeadingDeg, double maxYawPower) {
        while (opMode.opModeIsActive()
                && Math.abs(normDeg(targetHeadingDeg - getHeading())) > HEADING_THRESHOLD) {
            double yaw = getSteeringCorrection(targetHeadingDeg, P_YAW_GAIN);
            yaw = Range.clip(yaw, -Math.abs(maxYawPower), Math.abs(maxYawPower));
            teleOpRobotCentric(0, 0, yaw);
            opMode.idle();
        }
        teleOpRobotCentric(0, 0, 0);
    }

    public void holdHeading(double targetHeadingDeg, double seconds) {
        double end = runtime.seconds() + seconds;
        while (opMode.opModeIsActive() && runtime.seconds() < end) {
            double yaw = getSteeringCorrection(targetHeadingDeg, P_YAW_GAIN);
            teleOpRobotCentric(0, 0, yaw);
            opMode.idle();
        }
        teleOpRobotCentric(0, 0, 0);
    }

    public double getHeading() {
        return heading.get();
    }

    public void applyVisionYawCorrection(double visionHeadingDeg, double trust) {
        double err = normDeg(visionHeadingDeg - heading.getRawImu());
        heading.blendWithVision(err, trust);
    }

    private void setRunToPosition(DcMotor m, int target) {
        m.setTargetPosition(target);
        m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void setAllPowers(double p) {
        frontLeftDrive.setPower(p);
        frontRightDrive.setPower(p);
        backLeftDrive.setPower(p);
        backRightDrive.setPower(p);
    }

    private static double applyDeadband(double v) {
        return (Math.abs(v) < DEADBAND) ? 0.0 : v;
    }

    public double getSteeringCorrection(double targetHeading, double proportionalGain) {
        double headingError = normDeg(targetHeading - getHeading());
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    private static double normDeg(double a) {
        while (a > 180) a -= 360;
        while (a <= -180) a += 360;
        return a;
    }

    private class HeadingSubsystem {
        private double rawDeg = 0.0;
        private double offsetDeg = 0.0;

        void init() { }

        void update() {
            YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
            rawDeg = ypr.getYaw(AngleUnit.DEGREES);
        }

        double getRawImu() {
            return rawDeg;
        }

        double get() {
            return normDeg(rawDeg + offsetDeg);
        }

        void reset() {
            imu.resetYaw();
            offsetDeg = 0.0;
        }

        void blendWithVision(double errDeg, double alpha) {
            alpha = Range.clip(alpha, 0.0, 1.0);
            offsetDeg = normDeg(offsetDeg + alpha * errDeg);
        }
    }

    private class DriveSubsystem {
        private double prevAx = 0, prevLat = 0, prevYaw = 0;

        void init() { }

        void update() { }

        void stopAll() {
            setAllPowers(0);
        }

        void driveRobotCentric(double axialCmd, double lateralCmd, double yawCmd) {
            double axial   = slew(axialCmd,   prevAx,  SLEW_MAX_AXIAL);
            double lateral = slew(lateralCmd, prevLat, SLEW_MAX_LATERAL);
            double yaw     = slew(yawCmd,     prevYaw, SLEW_MAX_YAW);
            prevAx = axial; prevLat = lateral; prevYaw = yaw;

            double fl = axial + lateral + yaw;
            double fr = axial - lateral - yaw;
            double bl = axial - lateral + yaw;
            double br = axial + lateral - yaw;

            double max = Math.max(1.0, Math.max(Math.abs(fl),
                             Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
            fl /= max; fr /= max; bl /= max; br /= max;

            frontLeftDrive.setPower(fl);
            frontRightDrive.setPower(fr);
            backLeftDrive.setPower(bl);
            backRightDrive.setPower(br);
        }

        void driveFieldCentric(double axialCmd, double lateralCmd, double yawCmd, double headingDeg) {
            double axial   = slew(axialCmd,   prevAx,  SLEW_MAX_AXIAL);
            double lateral = slew(lateralCmd, prevLat, SLEW_MAX_LATERAL);
            double yaw     = slew(yawCmd,     prevYaw, SLEW_MAX_YAW);
            prevAx = axial; prevLat = lateral; prevYaw = yaw;

            double h = Math.toRadians(-headingDeg);
            double rotX = axial   * Math.cos(h) - lateral * Math.sin(h);
            double rotY = axial   * Math.sin(h) + lateral * Math.cos(h);

            driveRobotCentric(rotY, rotX, yaw);
        }

        private double slew(double target, double current, double maxDelta) {
            double delta = Range.clip(target - current, -maxDelta, maxDelta);
            return current + delta;
        }
    }
}
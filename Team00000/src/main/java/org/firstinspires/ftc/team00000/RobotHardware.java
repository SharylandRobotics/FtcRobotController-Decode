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

/**
 * RobotHardware (mentor-level)
 *
 * Design intent:
 * – Provide a thin, testable hardware layer with explicit lifestyle hooks.
 * – Drive math is centralized: field-centric using IMU heading from HeadingSubsystem.
 * – Vision can "blend" heading via a small offset without owning the drive.
 *
 * Notes:
 * – All public statics are Dashboard-tunable (@Config).
 * – Slew limiting smooths driver inputs; heading controllers use same path for simplicity.
 * – RUN_TO_POSITION straight-line auto adds proportional yaw correction to stay on heading.
 */
@Config
public class RobotHardware {

    // ───────────────── Tunables / thresholds (FTC Dashboard) ─────────────────

    /** P-terms used by simple proportion controllers (unit: output per degree). */
    public static double P_AXIAL_GAIN = 0.02;   // used for forward/back corrections when desired
    public static double P_LATERAL_GAIN = 0.02; // used for strafe corrections when desired
    public static double P_YAW_GAIN = 0.02;     // heading hold / turns

    /** Acceptable heading error for "on target" (deg). */
    public static double HEADING_THRESHOLD = 1.0;

    /** Input shaping; helps with traction/brownout and driver feel. */
    public static double DEADBAND = 0.05;
    public static double SLEW_MAX_AXIAL = 0.06;     // per loop step
    public static double SLEW_MAX_LATERAL = 0.06;   // per loop step
    public static double SLEW_MAX_YAW = 0.05;       // per loop step

    // Geometry/encoder constants (tune in Dashboard if wheel size or gearing changes)
    public static final double COUNTS_PER_MOTOR_REV = 537.7; // goBILDA 435 rmp
    public static final double DRIVE_GEAR_REDUCTION = 1.0;
    public static final double WHEEL_DIAMETER_INCHES = 3.78;

    /** Derived: ticks per inch (final so it matches your current code; consider a method for live tuning). */
    public static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    // ───────────────── FTC runtime wiring ─────────────────

    private final LinearOpMode opMode;

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private IMU imu;

    /** IMU source + vision blend lives here. */
    private final HeadingSubsystem heading = new HeadingSubsystem();
    /** Input shaping + power mixing lives here.*/
    private final DriveSubsystem drive = new DriveSubsystem();

    private final ElapsedTime runtime = new ElapsedTime();

    public RobotHardware(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    // ───────────────── Lifecycle (call from OpModes) ─────────────────

    /** Hardware map + base modes. Keep ALL hardwareMap.get(...) calls here (FTC best practice). */
    public void init() {
        frontLeftDrive = opMode.hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = opMode.hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = opMode.hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = opMode.hardwareMap.get(DcMotor.class, "backRightDrive");

        // Direction so +power drives forward for all four wheels.
        frontLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Safe idle behaviorl use encoders only when need for distance drive
        for (DcMotor m : new DcMotor[]{frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // IMU configuration must match physical hub mount or field-centric will drift
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

    /** Called once after start. Reset yaw reference for predictable field-centric. */
    public void start() {
        heading.reset();
        runtime.reset();
    }

    /** Per-loop updates; safe place for sensor reads & future closed-loop tasks. */
    public void periodic() {
        heading.update();
        drive.update();
    }

    /** Always stop motors on exit (TeleOp + Auto). */
    public void stop() {
        drive.stopAll();
    }

    // ───────────────── TeleOp drive API ─────────────────

    /** Robot-frame drive: forward is robot's front. */
    public void teleOpRobotCentric(double axial, double lateral, double yaw) {
        drive.driveRobotCentric(applyDeadband(axial), applyDeadband(lateral), applyDeadband(yaw));
    }

    /** Field-frame drive: forward is away from alliance wall, regardless of robot yaw. */
    public void teleOpFieldCentric(double axial, double lateral, double yaw) {
        drive.driveFieldCentric(applyDeadband(axial), applyDeadband(lateral), applyDeadband(yaw), getHeading());
    }

    // ───────────────── Simple Auto helpers ─────────────────

    /**
     * Drive a straight distance (inches) using RUN_TO_POSITION on all wheels,
     * adding proportional yaw correction to hold a desired heading.
     * @param inches     distance (+ forward / - backward)
     * @param maxPower   drive power magnitude (0..1)
     * @param headingDeg heading to hold (deg, absolute in current yaw frome)
     */
    public void autoRobotCentric(double inches, double maxPower, double headingDeg) {
        int move = (int) Math.round(inches * COUNTS_PER_INCH);

        setRunToPosition(frontLeftDrive,  frontLeftDrive.getCurrentPosition()  + move);
        setRunToPosition(frontRightDrive, frontRightDrive.getCurrentPosition() + move);
        setRunToPosition(backLeftDrive,   backLeftDrive.getCurrentPosition()   + move);
        setRunToPosition(backRightDrive,  backRightDrive.getCurrentPosition()  + move);

        setAllPowers(maxPower);
        // NOTE: for competitions, consider adding a timeout + isStopRequested() guard here.
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
        // Return to open-loop after the move competes.
        for (DcMotor m : new DcMotor[]{frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /** Closed-loop in-place turn to an absolute heading. */
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

    /** Stabilize heading for a duration; helpful after a turn to let the bot settle. */
    public void holdHeading(double targetHeadingDeg, double seconds) {
        double end = runtime.seconds() + seconds;
        while (opMode.opModeIsActive() && runtime.seconds() < end) {
            double yaw = getSteeringCorrection(targetHeadingDeg, P_YAW_GAIN);
            teleOpRobotCentric(0, 0, yaw);
            opMode.idle();
        }
        teleOpRobotCentric(0, 0, 0);
    }

    /** Current blended heading (IMU + any vision offset), degrees in (-180, 180]. */
    public double getHeading() {
        return heading.get();
    }

    /**
     * Vision seam: blend a small fraction of an absolute vision heading into our IMU frame.
     * Pass a small trust (e.g., 0.05 each loop) to gently nudge drift without jumps.
     * */
    public void applyVisionYawCorrection(double visionHeadingDeg, double trust) {
        double err = normDeg(visionHeadingDeg - heading.getRawImu());
        heading.blendWithVision(err, trust);
    }
    // ───────────────── Internal helpers ─────────────────
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
    /** P-control steering toward target heading; output in [-1,1]. */
    public double getSteeringCorrection(double targetHeading, double proportionalGain) {
        double headingError = normDeg(targetHeading - getHeading());
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /** Normalize an angle into (-180, 180] to ensure shortest-arc turns. */
    private static double normDeg(double a) {
        while (a > 180) a -= 360;
        while (a <= -180) a += 360;
        return a;
    }

    // ───────────────── Subsystems ─────────────────

    /**
     * HeadingSubsystem
     * – Reads IMU yaw each loop.
     * – Maintains a small offset that can be blended from vision to correct drift.
     */
    private class HeadingSubsystem {
        private double rawDeg = 0.0;    // IMU yaw as-read
        private double offsetDeg = 0.0; // blended correction

        void init() { }

        void update() {
            YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
            rawDeg = ypr.getYaw(AngleUnit.DEGREES);
        }

        /** Raw IMU yaw (deg), no offset applied. */
        double getRawImu() {
            return rawDeg;
        }

        /** Blended heading (deg.). */
        double get() {
            return normDeg(rawDeg + offsetDeg);
        }

        /** Reset IMU reference and clear offsets (use at start or to re-zero). */
        void reset() {
            imu.resetYaw();
            offsetDeg = 0.0;
        }

        /** Exponential blend: offset += alpha * error, Keep alpha small for smoothness. */
        void blendWithVision(double errDeg, double alpha) {
            alpha = Range.clip(alpha, 0.0, 1.0);
            offsetDeg = normDeg(offsetDeg + alpha * errDeg);
        }
    }

    /**
     * DriveSubsystem
     * – Applies deadband + slew limiting to commands.
     * – Computes mecanum wheel powers (robot-frame).
     * – Field-centric rotates sticks by -heading then reuses robot-centric path.
     */
    private class DriveSubsystem {
        private double prevAx = 0, prevLat = 0, prevYaw = 0;

        void init() { }
        void update() { }

        void stopAll() {
            setAllPowers(0);
        }

        /** Robot-frame drive with input shaping and power normalization. */
        void driveRobotCentric(double axialCmd, double lateralCmd, double yawCmd) {
            double axial   = slew(axialCmd,   prevAx,  SLEW_MAX_AXIAL);
            double lateral = slew(lateralCmd, prevLat, SLEW_MAX_LATERAL);
            double yaw     = slew(yawCmd,     prevYaw, SLEW_MAX_YAW);
            prevAx = axial; prevLat = lateral; prevYaw = yaw;

            double fl = axial + lateral + yaw;
            double fr = axial - lateral - yaw;
            double bl = axial - lateral + yaw;
            double br = axial + lateral - yaw;

            // Normalize to keep |power| ≤ 1
            double max = Math.max(1.0, Math.max(Math.abs(fl),
                             Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
            fl /= max; fr /= max; bl /= max; br /= max;

            frontLeftDrive.setPower(fl);
            frontRightDrive.setPower(fr);
            backLeftDrive.setPower(bl);
            backRightDrive.setPower(br);
        }

        /** Field-frame drive: rotate sticks by -heading, then use robot-centric mix. */
        void driveFieldCentric(double axialCmd, double lateralCmd, double yawCmd, double headingDeg) {
            double axial   = slew(axialCmd,   prevAx,  SLEW_MAX_AXIAL);
            double lateral = slew(lateralCmd, prevLat, SLEW_MAX_LATERAL);
            double yaw     = slew(yawCmd,     prevYaw, SLEW_MAX_YAW);
            prevAx = axial; prevLat = lateral; prevYaw = yaw;

            double h = Math.toRadians(-headingDeg);
            double rotX = axial   * Math.cos(h) - lateral * Math.sin(h);
            double rotY = axial   * Math.sin(h) + lateral * Math.cos(h);

            // Reuse robot-centric path for final mixing/normalization.
            driveRobotCentric(rotY, rotX, yaw);
        }

        /** First-order rate limiter to smooth step changes from sticks/autonomous*/
        private double slew(double target, double current, double maxDelta) {
            double delta = Range.clip(target - current, -maxDelta, maxDelta);
            return current + delta;
        }
    }
}
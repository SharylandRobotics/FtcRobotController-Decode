package org.firstinspires.ftc.team00000;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

/**
 * RobotHardware
 * Thin hardware layer with clear lifecycle hooks. Field-centric drive uses IMU.
 * AprilTag vision fills a pose cache (x,y,z,yawDeg,range,tagId,hasFix).
 * Tunables are exposed in FTC Dashboard (@Config).
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
    public static final double COUNTS_PER_MOTOR_REV = 537.7; // goBILDA 435 rpm
    public static final double DRIVE_GEAR_REDUCTION = 1.0;
    public static final double WHEEL_DIAMETER_INCHES = 3.78;

    /** Derived: ticks per inch (final so it matches your current code; consider a method for live tuning). */
    public static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    public static boolean VISION_HEADING_BLEND_ENABLED = true;
    public static double VISION_HEADING_TRUST = 0.05; // 0.02-0.08 is typical
    public static boolean VISION_DEBUG_PASS_THROUGH = false;
    public static boolean VISION_YAW_INVERT = true; // some SDK builds report camera yaw; invert to get robot yaw
    public static double VISION_YAW_OFFSET_DEG = 0.0; // manual trim to align vision yaw to IMU

    // ───────────────── FTC runtime wiring ─────────────────

    private final LinearOpMode opMode;

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private IMU imu;

    /** IMU source + vision blend lives here. */
    private final HeadingSubsystem heading = new HeadingSubsystem();
    /** Input shaping + power mixing lives here.*/
    private final DriveSubsystem drive = new DriveSubsystem();

    /** Vision portal + AprilTag */
    private final VisionSubsystem vision = new VisionSubsystem();

    /** Simple pose cache exposed to other subsystems */
    public static class FieldPose {
        // AprilTag convention (meters, degrees): x=+right, y=+forward, z=+up, yaw around +z
        public double x = 0, y = 0, z = 0;   // meters
        public double yawDeg = 0;            // degrees
        public boolean hasFix = false;
        public int tagId = -1;
        public double rangeM = Double.NaN;
    }
    private final FieldPose pose = new FieldPose();

    /** Public accessors for other code (shooter / turret / autonomous) */
    public FieldPose getFieldPose() { return pose; }
    public boolean hasTagFix() { return pose.hasFix; }
    public double getRangeToGoalM() { return pose.rangeM; } // used by ShooterLUT

    /** Vision-only yaw (deg), or 0 if no fix. */
    public double getVisionYawDeg() { return pose.hasFix ? pose.yawDeg : 0.0; }
    /** Vision translation (m) in AprilTag convention; returns (0,0,0) if no fix. */
    public double[] getVisionXYZ() { return new double[]{ pose.hasFix ? pose.x : 0.0, pose.hasFix ? pose.y : 0.0, pose.hasFix ? pose.z : 0.0 }; }

    /** Returns suggested shooter RPM based on current tag range, or 0 if no fix. */
    public double getSuggestedShooterRpm() {
        double d = getRangeToGoalM();
        if (!hasTagFix() || Double.isNaN(d)) return 0.0;
        return org.firstinspires.ftc.team00000.subsystems.ShooterLUT.rpmForDistance(d);
    }

    /** Dump current vision info to telemetry (call from OpMode loop). */
    public void telemetryVision(Telemetry tel) {
        if (tel == null) return;
        if (!hasTagFix()) {
            tel.addLine("Tag: none");
            return;
        }
        tel.addData("Tag", "#%d", pose.tagId);
        tel.addData("Range (m)", "%.2f", pose.rangeM);
        tel.addData("Yaw (deg)", "%.1f", pose.yawDeg);
    }

    /** Nudge robot until tag range ≈ targetM; returns when within tol or no fix. */
    public void assistDriveToTagRange(double targetM, double tolM, double maxPower) {
        if (!hasTagFix() || maxPower <= 0) return;
        maxPower = Math.abs(Range.clip(maxPower, 0, 1));
        if (!hasTagFix()) return;
        double targetHeading = getHeading();
        double err = getRangeToGoalM() - targetM;
        while (opMode.opModeIsActive() && hasTagFix() && Math.abs(err) > tolM) {
            double axial = Range.clip(err * 0.7, -Math.abs(maxPower), Math.abs(maxPower));
            double yaw   = getSteeringCorrection(targetHeading, P_YAW_GAIN);
            teleOpRobotCentric(axial, 0, yaw);
            opMode.idle();
            err = getRangeToGoalM() - targetM;
        }
        teleOpRobotCentric(0, 0, 0);
    }

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

        // Safe idle behavior: use encoders only when need for distance drive
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
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);

        heading.init();
        drive.init();
        vision.init();
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
        vision.update();
    }

    /** Always stop motors on exit (TeleOp + Auto). */
    public void stop() {
        drive.stopAll();
        vision.shutdown();
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
     * @param headingDeg heading to hold (deg, absolute in current yaw frame)
     */
    public void autoRobotCentric(double inches, double maxPower, double headingDeg) {
        int move = (int) Math.round(inches * COUNTS_PER_INCH);
        setRunToPosition(frontLeftDrive,  frontLeftDrive.getCurrentPosition()  + move);
        setRunToPosition(frontRightDrive, frontRightDrive.getCurrentPosition() + move);
        setRunToPosition(backLeftDrive,   backLeftDrive.getCurrentPosition()   + move);
        setRunToPosition(backRightDrive,  backRightDrive.getCurrentPosition()  + move);

        setAllPowers(maxPower);

        double timeoutS = Math.max(1.5, Math.abs(inches) * 0.25); // simple heuristic
        double end = runtime.seconds() + timeoutS;

        while (opMode.opModeIsActive()
                && runtime.seconds() < end
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

        // Prefer ftcPose (SDK units/axes); fall back to robotPose or bearing+range
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

        // Translation camera→tag (m) — AprilTag convention: x=+right, y=+forward, z=+up
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

            final double h = Math.toRadians(-headingDeg);
            final double c = Math.cos(h), s = Math.sin(h);
            double rotX = axial * c - lateral * s;
            double rotY = axial * s + lateral * c;

            // Reuse robot-centric path for final mixing/normalization.
            driveRobotCentric(rotY, rotX, yaw);
        }

        /** First-order rate limiter to smooth step changes from sticks/autonomous*/
        private double slew(double target, double current, double maxDelta) {
            double delta = Range.clip(target - current, -maxDelta, maxDelta);
            return current + delta;
        }
    }

    /** AprilTag vision runtime: pulls detections, computes coarse field pose, and blends heading */
    private class VisionSubsystem {

        private VisionPortal portal;
        private AprilTagProcessor tags;

        // Camera intrinsics (use ConceptAprilTag* as reference) – Tune from calibration
        // These defaults let you start; replicate with your calibrated values.
        private static final double FX = 921.31; // pixels
        private static final double FY = 917.70;
        private static final double CX = 689.03;
        private static final double CY = 372.06;

        void init() {
            tags = new AprilTagProcessor.Builder()
                    .setLensIntrinsics(FX, FY, CX, CY)
                    .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                    .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                    .build();

            // Enable camera streaming for FTC Dashboard (instead of DS preview)
            portal = new VisionPortal.Builder()
                    .setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(tags)
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .setCameraResolution(new android.util.Size(1280,800))
                    .build();
            // Stream to FTC Dashboard (192.168.43.1) — modest FPS to keep CPU/GPU headroom
            try { com.acmerobotics.dashboard.FtcDashboard.getInstance().startCameraStream(portal, 12); } catch (Throwable ignored) { }
        }
        // Vision yaw (deg): from ftcPose.yaw if present; else from -bearing
        void update() {
            if (portal == null || tags == null) return;

            List<AprilTagDetection> dets = tags.getDetections();
            // Explicitly reset pose fields for telemetry every frame
            pose.hasFix = false; // reset each frame
            pose.tagId = -1;
            pose.rangeM = Double.NaN;

            AprilTagDetection best = pickBest(dets);
            if (best == null) return; // pose already reset above

            // Prefer SDK-provided ftcPose (already in meters due to setOutputUnits). Fallback to robotPose.
            double range;
            double bearingRobotDeg;
            if (best.ftcPose != null && !Double.isNaN(best.ftcPose.range) && best.ftcPose.range > 0) {
                range = best.ftcPose.range;
                bearingRobotDeg = best.ftcPose.bearing;
            } else if (best.robotPose != null && best.robotPose.getPosition() != null) {
                double tx = best.robotPose.getPosition().x; // +right (m)
                double ty = best.robotPose.getPosition().y; // +forward (m)
                range = Math.hypot(tx, ty);
                bearingRobotDeg = Math.toDegrees(Math.atan2(tx, ty));
            } else {
                // No usable pose
                pose.hasFix = false;
                pose.tagId = -1;
                pose.rangeM = Double.NaN;
                return;
            }

            // Populate translation (camera -> tag) in meters using AprilTag convention
            if (best.ftcPose != null && !Double.isNaN(best.ftcPose.x) && !Double.isNaN(best.ftcPose.y)) {
                pose.x = best.ftcPose.x; // +right (m)
                pose.y = best.ftcPose.y; // +forward (m)
                pose.z = !Double.isNaN(best.ftcPose.z) ? best.ftcPose.z : 0.0; // +up (m)
            } else if (best.robotPose != null && best.robotPose.getPosition() != null) {
                pose.x = best.robotPose.getPosition().x;
                pose.y = best.robotPose.getPosition().y;
                try { pose.z = best.robotPose.getPosition().z; } catch (Throwable ignored) { pose.z = 0.0; }
            } else {
                // Reconstruct x/y from bearing + range if no pose vector is available
                double rad = Math.toRadians(bearingRobotDeg);
                pose.x = Math.sin(rad) * range; // +right
                pose.y = Math.cos(rad) * range; // +forward
                pose.z = 0.0;
            }

            // Compute a direct vision yaw (no IMU blending): prefer orientation yaw, fallback to -bearing
            double headingVisionDeg;
            if (best.ftcPose != null && !Double.isNaN(best.ftcPose.yaw)) {
                double camYaw = best.ftcPose.yaw; // degrees, per setOutputUnits
                headingVisionDeg = (VISION_YAW_INVERT ? -camYaw : camYaw) + VISION_YAW_OFFSET_DEG;
            } else {
                headingVisionDeg = -bearingRobotDeg; // proxy from geometry
            }

            // Publish minimal, pure-vision pose for telemetry
            pose.tagId = best.id;
            pose.rangeM = range;
            pose.yawDeg = normDeg(headingVisionDeg);
            pose.hasFix = true;
        }

        void shutdown() {
            if (portal != null) {
                try {
                    com.acmerobotics.dashboard.FtcDashboard.getInstance().stopCameraStream();
                } catch (Throwable ignored) { }
                portal.close();
                portal = null;
                tags = null;
            }
        }

        private static final double MIN_RANGE_M = 0.20; // too close = likely wrong
        private static final double MAX_RANGE_M = 7.0; // beyond reasonable FTC viewing

        // Choose nearest valid tag (prefers robotPose; else ftcPose range). Filters outliers.
        private AprilTagDetection pickBest(List<AprilTagDetection> dets) {
            if (dets == null || dets.isEmpty()) return null;

            // Optional: In debug pass-through, just return the first detection with metadata
            if (VISION_DEBUG_PASS_THROUGH) {
                for (AprilTagDetection d : dets) {
                    if (d.metadata != null) return d;
                }
            }

            AprilTagDetection best = null;
            for (AprilTagDetection d : dets) {
                if (d.metadata == null) continue; // require known tag
                // Prefer robotPose when available, otherwise fall back to ftcPose for range
                double tx, ty;
                boolean hasRobotPose = (d.robotPose != null);
                if (hasRobotPose) {
                    tx = d.robotPose.getPosition().x;
                    ty = d.robotPose.getPosition().y;
                } else if (d.ftcPose != null && !Double.isNaN(d.ftcPose.range) && d.ftcPose.range > 0 && !Double.isNaN(d.ftcPose.bearing)) {
                    // Reconstruct x/y from bearing+range
                    tx = Math.sin(Math.toRadians(d.ftcPose.bearing)) * d.ftcPose.range;
                    ty = Math.cos(Math.toRadians(d.ftcPose.bearing)) * d.ftcPose.range;
                } else {
                    continue; // no usable pose
                }
                double r = Math.hypot(tx, ty);
                if (!VISION_DEBUG_PASS_THROUGH) {
                    if (r < MIN_RANGE_M || r > MAX_RANGE_M) continue;
                }
                if (best == null) best = d;
                else {
                    double br;
                    if (best.robotPose != null && best.robotPose.getPosition() != null) {
                        br = Math.hypot(best.robotPose.getPosition().x, best.robotPose.getPosition().y);
                    } else if (best.ftcPose != null && !Double.isNaN(best.ftcPose.range) && best.ftcPose.range > 0) {
                        br = best.ftcPose.range;
                    } else {
                        br = Double.POSITIVE_INFINITY;
                    }
                    if (r < br) best = d; // prefer nearer
                }
            }

            // If all filtered out, but we had detections, return first with metadata as a last resort
            if (best == null) {
                for (AprilTagDetection d : dets) if (d.metadata != null) return d;
            }
            return best;
        }
    }
}
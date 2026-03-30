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

package org.firstinspires.ftc.team00000;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

// Reference TeleOp for Team00000.
// Controls:
// - Drive: left stick translate, right stick rotate, left bumper precision mode.
// - Vision assist: hold right bumper for AprilTag goal approach.
// - Shooter: X toggles spin-up, Y starts timed burst feed.
// - Transfer/stoppers: d-pad and A/B manage intake, reverse, and gate state.
@Config
@TeleOp(name = "Drive TeleOp", group = "opMode")

public class DriveTeleOp extends LinearOpMode {

    // Robot subsystem wrapper (drive, shooter, transfer, vision).
    RobotHardware robot = new RobotHardware(this);

    // Shooter tuning exposed through FTC Dashboard.
    public static double shooterFar = 0.38;
    public static double shooterMaxRpm = 0.0;
    public static double shooterKp = 0.3;
    public static double shooterKi = 0.0;
    public static double shooterKd = 0.0;

    // Cached PIDF values to avoid writing coefficients every loop iteration.
    private double lastEffMaxRpm = Double.NaN;
    private double lastShooterKp = Double.NaN;
    private double lastShooterKi = Double.NaN;
    private double lastShooterKd = Double.NaN;

    // Current mechanism outputs (also visible in Dashboard).
    public static double stopperPower = 0.0;
    public static double transferPower = 0.0;

    // Previous-frame gamepad state used for edge detection.
    private boolean prevUp = false;
    private boolean prevDown = false;
    private boolean prevLeft = false;
    private boolean prevRight = false;
    private boolean prevA = false;
    private boolean prevX = false;
    private boolean prevY = false;
    private boolean prevL3 = false;

    // Runtime operator state.
    private boolean shooterEnabled = false;
    private boolean intakeMode = false;
    private boolean launchMode = false;
    private long launchStartMs = 0;
    private long readySinceMs = 0;
    private long feedStartMs = 0;
    private long feedAccumulatedMs = 0;
    private long lastLaunchLoopMs = 0;

    // Shot-profile multipliers used during burst feed.
    public static double SHOT1_MULTIPLIER = 1.00;
    public static double SHOT2_MULTIPLIER = 1.03;
    public static double SHOT3_MULTIPLIER = 1.06;

    // 0-based selected shot index.
    private int shotIndex = 0;
    // Latched at burst start so profile math stays stable during feed.
    private int firingShotIndex = 0;

    // Shooter speed sag capture (ticks/sec magnitude).
    private double preFeedTicksPerSecond = Double.NaN;
    private double minimumFeedTicksPerSecond = Double.NaN;

    // Drive mode selection (field-centric vs robot-centric).
    private enum DriveMode { FIELD, ROBOT }
    private DriveMode driveMode = DriveMode.FIELD;

    // Burst timing (milliseconds).
    public static int LAUNCH_SPOOL_MS = 0;
    public static int LAUNCH_FEED_MS = 1200;
    // Number of artifacts launched per Y burst.
    public static int LAUNCH_ARTIFACTS_PER_BURST = 3;

    public static double SHOOT_READY_PCT_TOL = 0.20;
    public static double SHOOT_FEED_PCT_TOL = 0.10;
    public static int SHOOT_READY_TIMEOUT_MS = 900;
    public static int FIRE_SETTLE_MS = 80;
    public static boolean ALLOW_FEED_ON_TIMEOUT = true;
    public static double SHOOTER_FEED_BOOST = 0.10;
    public static boolean IMMEDIATE_LAUNCH_ON_Y = true;
    public static double SHOOT_READY_MIN_TPS = 470.0;
    public static double SHOOT_FEED_MIN_TPS = 430.0;
    public static double SHOOTER_LAUNCH_MIN_CMD = 0.45;

    // Mechanism power presets.
    public static double TRANSFER_FWD = 0.70;
    public static double TRANSFER_REV = -0.20;
    public static double STOPPER_OPEN = 0.80;
    public static double STOPPER_CLOSED = 0.00;
    public static boolean TELEMETRY_VERBOSE = false;

    @Override
    public void runOpMode() {

        double axial;
        double lateral;
        double yaw;

        // Initialize hardware and telemetry sinks (Driver Station + Dashboard).
        robot.init();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);

        while(opModeInInit()) {
            // Pre-start checks: verify heading updates and vision is initialized.
            telemetry.addData("Status", "Hardware Initialized");
            telemetry.addData("Heading", "%4.0f", robot.getHeading());
            if (TELEMETRY_VERBOSE) {
                telemetry.addData("Vision", "Ready (AprilTag)");
                telemetry.addData("Mode", "INIT");
                telemetry.addData("Obelisk", robot.hasObeliskMotif() ? String.format("%s (ID %s)",
                        robot.getObeliskMotif(), robot.getObeliskTagId()) : "–");
            }
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Refresh AprilTag detections (used for assist + telemetry).
            robot.updateAprilTagDetections();

            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            boolean left = gamepad1.dpad_left;
            boolean right = gamepad1.dpad_right;
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;
            boolean y = gamepad1.y;
            boolean l3 = gamepad1.left_stick_button;

            boolean upEdge = up && !prevUp;
            boolean downEdge = down && !prevDown;
            boolean leftEdge = left && !prevLeft;
            boolean rightEdge = right && !prevRight;
            boolean aEdge = a && !prevA;
            boolean xEdge = x && !prevX;
            boolean yEdge = y && !prevY;
            boolean l3Edge = l3 && !prevL3;

            // Left Stick: toggle drive mode.
            if (l3Edge) {
                driveMode = (driveMode == DriveMode.FIELD) ? DriveMode.ROBOT : DriveMode.FIELD;
                robot.resetYaw();
            }

            // D-pad UP/DOWN: transfer control.
            if (upEdge) {
                transferPower = TRANSFER_FWD;
                intakeMode = true;
                launchMode = false;
            }
            if (downEdge) {
                transferPower = TRANSFER_REV;
                intakeMode = true;
                launchMode = false;
            }

            // D-pad LEFT: safe stop.
            if (leftEdge) {
                transferPower = 0.0;
                stopperPower = STOPPER_CLOSED;
                intakeMode = false;
                launchMode = false;
                shotIndex = 0;
            }

            // D-pad RIGHT: toggle stoppers.
            if (rightEdge) {
                stopperPower = (stopperPower == STOPPER_OPEN) ? STOPPER_CLOSED : STOPPER_OPEN;
            }

            // A: toggle intake mode.
            if (aEdge) {
                intakeMode = !intakeMode;
                launchMode = false;
                if (intakeMode) {
                    transferPower = TRANSFER_FWD;
                } else {
                    transferPower = 0.0;
                }
                stopperPower = STOPPER_CLOSED;
                shotIndex = 0;
            }

            // B (hold): quick reverse to clear jams.
            if (b) {
                transferPower = TRANSFER_REV;
                stopperPower = STOPPER_CLOSED;
                intakeMode = false;
                launchMode = false;
            }

            // X: toggle shooter spin-up.
            if (xEdge) {
                shooterEnabled = !shooterEnabled;
                if (!shooterEnabled) {
                    launchMode = false;
                    stopperPower = STOPPER_CLOSED;
                    transferPower = 0.0;
                    intakeMode = false;
                    shotIndex = 0;
                }
            }

            // Y: burst feed for a fixed time window; requires shooter enabled.
            if (yEdge && shooterEnabled) {
                launchMode = true;
                intakeMode = false;
                launchStartMs = System.currentTimeMillis();
                readySinceMs = 0;
                feedStartMs = 0;
                feedAccumulatedMs = 0;
                lastLaunchLoopMs = launchStartMs;
                firingShotIndex = 0;
                preFeedTicksPerSecond = Double.NaN;
                minimumFeedTicksPerSecond = Double.NaN;
            }

            if(launchMode && shooterEnabled) {
                long now = System.currentTimeMillis();
                long t = now - launchStartMs;
                long dt = Math.max(0, now - lastLaunchLoopMs);
                lastLaunchLoopMs = now;

                if (IMMEDIATE_LAUNCH_ON_Y) {
                    if (feedStartMs == 0) {
                        feedStartMs = now;
                        preFeedTicksPerSecond = Math.abs(robot.getShooterVelocityPerSecond());
                        minimumFeedTicksPerSecond = preFeedTicksPerSecond;
                    }
                    long totalFeedMs = (long) LAUNCH_FEED_MS * Math.max(1, LAUNCH_ARTIFACTS_PER_BURST);
                    if (feedAccumulatedMs < totalFeedMs) {
                        double currentTicksPerSecond = Math.abs(robot.getShooterVelocityPerSecond());
                        if (!Double.isNaN(minimumFeedTicksPerSecond)) {
                            minimumFeedTicksPerSecond = Math.min(minimumFeedTicksPerSecond, currentTicksPerSecond);
                        }
                        feedAccumulatedMs += dt;
                        stopperPower = STOPPER_OPEN;
                        transferPower = TRANSFER_FWD;
                    } else {
                        stopperPower = STOPPER_CLOSED;
                        transferPower = 0.0;
                        launchMode = false;
                        shotIndex = 0;
                        readySinceMs = 0;
                        feedStartMs = 0;
                        feedAccumulatedMs = 0;
                        lastLaunchLoopMs = 0;
                        preFeedTicksPerSecond = Double.NaN;
                        minimumFeedTicksPerSecond = Double.NaN;
                    }
                } else {
                    boolean atSpeed = isShooterReady(SHOOT_READY_PCT_TOL, SHOOT_READY_MIN_TPS);

                    if (atSpeed) {
                        if (readySinceMs == 0) readySinceMs = now;
                    } else {
                        readySinceMs = 0;
                    }

                    boolean readyStable = (readySinceMs != 0) && (now - readySinceMs >= FIRE_SETTLE_MS);
                    boolean timeoutHit = (t >= SHOOT_READY_TIMEOUT_MS);
                    boolean minSpoolMet = (t >= LAUNCH_SPOOL_MS);
                    boolean readyOrTimeout = (minSpoolMet && readyStable) || timeoutHit;

                    if (!readyOrTimeout) {
                        // Spool phase: keep stoppers closed and do not feed.
                        stopperPower = STOPPER_CLOSED;
                        transferPower = 0.0;
                        feedStartMs = 0;
                    } else {
                        // Start feeding exactly once.
                        if (feedStartMs == 0) {
                            feedStartMs = now;
                            preFeedTicksPerSecond = Math.abs(robot.getShooterVelocityPerSecond());
                            minimumFeedTicksPerSecond = preFeedTicksPerSecond;
                        }

                        long totalFeedMs = (long) LAUNCH_FEED_MS * Math.max(1, LAUNCH_ARTIFACTS_PER_BURST);
                        boolean atFeedSpeed = isShooterReady(SHOOT_FEED_PCT_TOL, SHOOT_FEED_MIN_TPS);
                        boolean allowFeedNow = atFeedSpeed || (timeoutHit && ALLOW_FEED_ON_TIMEOUT);
                        if (feedAccumulatedMs < totalFeedMs) {
                            double currentTicksPerSecond = Math.abs(robot.getShooterVelocityPerSecond());
                            if (!Double.isNaN(minimumFeedTicksPerSecond)) {
                                minimumFeedTicksPerSecond = Math.min(minimumFeedTicksPerSecond, currentTicksPerSecond);
                            }
                            if (allowFeedNow) {
                                feedAccumulatedMs += dt;
                                stopperPower = STOPPER_OPEN;
                                transferPower = TRANSFER_FWD;
                            } else {
                                // Hold feed closed until shooter speed recovers.
                                stopperPower = STOPPER_CLOSED;
                                transferPower = 0.0;
                            }
                        } else {
                            stopperPower = STOPPER_CLOSED;
                            transferPower = 0.0;
                            launchMode = false;
                            shotIndex = 0;
                            readySinceMs = 0;
                            feedStartMs = 0;
                            feedAccumulatedMs = 0;
                            lastLaunchLoopMs = 0;
                            preFeedTicksPerSecond = Double.NaN;
                            minimumFeedTicksPerSecond = Double.NaN;
                        }
                    }
                }
            } else {
                // Safety: keep stoppers closed while in-taking.
                if (intakeMode && !b) {
                    stopperPower = STOPPER_CLOSED;
                }

                readySinceMs = 0;
                feedStartMs = 0;
                feedAccumulatedMs = 0;
                lastLaunchLoopMs = 0;
                preFeedTicksPerSecond = Double.NaN;
                minimumFeedTicksPerSecond = Double.NaN;
            }

            prevUp = up;
            prevDown = down;
            prevLeft = left;
            prevRight = right;
            prevA = a;
            prevX = x;
            prevY = y;
            prevL3 = l3;

            // Slow mode scales driver inputs for precision.
            boolean slow = gamepad1.left_bumper;
            double scale = slow ? 0.4 : 1.0;

            axial = -gamepad1.left_stick_y * scale;
            lateral = gamepad1.left_stick_x * scale;
            yaw = gamepad1.right_stick_x * scale;

            // Use motor's configured maxRPM unless overridden from Dashboard.
            double effMaxRpm = (shooterMaxRpm > 0.0) ? shooterMaxRpm : robot.getShooterMaxRpm();
            if (Double.isNaN(lastEffMaxRpm)
                    || Math.abs(effMaxRpm - lastEffMaxRpm) > 1e-6
                    || Math.abs(shooterKp - lastShooterKp) > 1e-9
                    || Math.abs(shooterKi - lastShooterKi) > 1e-9
                    || Math.abs(shooterKd - lastShooterKd) > 1e-9) {
                robot.configureShooterVelocityPIDFForMaxRpm(effMaxRpm, shooterKp, shooterKi, shooterKd);
                lastEffMaxRpm = effMaxRpm;
                lastShooterKp = shooterKp;
                lastShooterKi = shooterKi;
                lastShooterKd = shooterKd;
            }
            double shooterBase = shooterEnabled ? shooterFar : 0.0;

            int effectiveShotIndex;
            if (launchMode && shooterEnabled && feedStartMs != 0) {
                long feedElapsed = System.currentTimeMillis() - feedStartMs;
                int step = (LAUNCH_FEED_MS > 0) ? (int) (feedElapsed / (long) LAUNCH_FEED_MS) : 0;
                step = Range.clip(step, 0, 2); // only 3 multipliers available
                effectiveShotIndex = firingShotIndex + step;
            } else {
                effectiveShotIndex = shotIndex;
            }
            effectiveShotIndex = Range.clip(effectiveShotIndex, 0, 2);

            double shotMultiplier = (effectiveShotIndex == 0) ? SHOT1_MULTIPLIER : (effectiveShotIndex == 1 ? SHOT2_MULTIPLIER : SHOT3_MULTIPLIER);
            double shooterApplied = shooterBase * shotMultiplier;
            if (launchMode && shooterEnabled) {
                shooterApplied = Range.clip(shooterApplied + SHOOTER_FEED_BOOST, 0.0, 1.0);
                shooterApplied = Math.max(shooterApplied, SHOOTER_LAUNCH_MIN_CMD);
            }

            robot.setShooterVelocityPercent(shooterApplied, effMaxRpm);

            // Vision values (for telemetry and driver assist).
            Integer goalId = robot.getGoalTagId();
            double range = robot.getGoalRangeIn();
            double bearing = robot.getGoalBearingDeg();
            double elevation = robot.getGoalElevationDeg();

            // Derived aim helpers (assumes camera pitched up 8°).
            double horizontal = (Double.isNaN(range) || Double.isNaN(bearing))
                    ? Double.NaN
                    : range * Math.cos(Math.toRadians(bearing));
            double aimAboveHorizontal = (Double.isNaN(elevation) ? Double.NaN : (8.0 + elevation));

            // RB (hold): AprilTag assist (range->drive, tagYaw->strafe, bearing->turn).
            boolean autoAssist = gamepad1.right_bumper;
            boolean didAuto = false;
            if (autoAssist) {
                // Refresh again right before assist step for best recency.
                robot.updateAprilTagDetections();
                didAuto = robot.autoDriveToGoalStep();
            }

            // Manual drive (routes to FIELD- or ROBOT-centric based on driveMode).
            if (!didAuto) {
                if (driveMode == DriveMode.FIELD) {
                    robot.driveFieldCentric(axial, lateral, yaw);
                } else {
                    robot.driveRobotCentric(axial, lateral, yaw);
                }
            }

            robot.setStopperPower(stopperPower);
            robot.setTransferPower(transferPower);

            telemetry.addData("Mode", slow ? "SLOW" : "NORMAL");
            telemetry.addData("Drive", driveMode == DriveMode.FIELD ? "FIELD" : "ROBOT");
            telemetry.addData("Heading", "%4.0f°", robot.getHeading());

            double targetTicksPerSecond = Math.abs(robot.getShooterTargetTicksPerSec());
            double currentTicksPerSecond = Math.abs(robot.getShooterVelocityPerSecond());
            double shooterSpeedPercent = (targetTicksPerSecond > 1.0) ? (currentTicksPerSecond / targetTicksPerSecond) * 100.0 : 0.0;

            int activeShotIndex;
            if (launchMode && shooterEnabled && feedStartMs != 0) {
                long feedElapsed = System.currentTimeMillis() - feedStartMs;
                int step = (LAUNCH_FEED_MS > 0) ? (int) (feedElapsed / (long) LAUNCH_FEED_MS) : 0;
                step = Range.clip(step, 0, 2);
                activeShotIndex = firingShotIndex + step;
            } else {
                activeShotIndex = shotIndex;
            }
            activeShotIndex = Range.clip(activeShotIndex, 0, 2);
            double activeShotMultiplier = (activeShotIndex == 0) ? SHOT1_MULTIPLIER : (activeShotIndex == 1 ? SHOT2_MULTIPLIER : SHOT3_MULTIPLIER);

            telemetry.addData("Shooter", shooterEnabled ? "ON" : "OFF");
            telemetry.addData("Launch", launchMode ? "ON" : "OFF");
            telemetry.addData("ShooterTPS", "%.0f", currentTicksPerSecond);
            telemetry.addData("Shot", "%d", (activeShotIndex + 1));
            telemetry.addData("AtSpeed", "%s",
                    isShooterReady(SHOOT_READY_PCT_TOL, SHOOT_READY_MIN_TPS) ? "YES" : "no",
                    SHOOT_READY_PCT_TOL * 100.0);
            telemetry.addData("FeedReady", "%s",
                    isShooterReady(SHOOT_FEED_PCT_TOL, SHOOT_FEED_MIN_TPS) ? "YES" : "no",
                    SHOOT_FEED_PCT_TOL * 100.0);

            if (TELEMETRY_VERBOSE) {
                telemetry.addData("Assist", autoAssist ? (didAuto ? "AUTO→TAG" : "NO TAG") : "MANUAL");
                telemetry.addData("DriveRaw", "ax=%.2f  lat=%.2f  yaw=%.2f", axial, lateral, yaw);
                telemetry.addData("ShooterCommand", "%.2f", shooterEnabled ? shooterFar : 0.0);
                telemetry.addData("ShooterCmdApplied", "%.2f", shooterEnabled ? (shooterFar * activeShotMultiplier) : 0.0);
                telemetry.addData("ShooterModelPercent", "=%.0f%%", shooterSpeedPercent);
                telemetry.addData("StopperPower", "%.2f", stopperPower);
                telemetry.addData("TransferPower", "%.2f", transferPower);
                telemetry.addData("Op", "shooter=%s intake=%s launch=%s",
                        shooterEnabled ? "ON" : "OFF",
                        intakeMode ? "ON" : "OFF",
                        launchMode ? "ON" : "OFF");

                if (!Double.isNaN(preFeedTicksPerSecond) && !Double.isNaN(minimumFeedTicksPerSecond) && preFeedTicksPerSecond > 1.0) {
                    double shooterSagPercent = (1.0 - (minimumFeedTicksPerSecond / preFeedTicksPerSecond)) * 100.0;
                    telemetry.addData("ShooterSag", "=%.0f%%", shooterSagPercent);
                } else {
                    telemetry.addData("ShooterSag", "–");
                }
                telemetry.addData("ShooterMax", "%.0f t/s (model)", robot.getShooterMaxTicksPerSec());

                String motif = robot.hasObeliskMotif() ? String.format("%s (ID %s)", robot.getObeliskMotif(), robot.getObeliskTagId()) : "–";
                telemetry.addData("Obelisk", motif);
                telemetry.addData("Goal", (goalId != null) ? goalId : "–");
                telemetry.addData("Pose", "rng=%.1f in  brg=%.1f°  elev=%.1f°", range, bearing, elevation);
                String aimAboveText = Double.isNaN(aimAboveHorizontal) ? "–" : String.format("%.1f°", aimAboveHorizontal);
                telemetry.addData("Aim",  "horiz=%.1f in  aboveHorizontal=%s", horizontal, aimAboveText);
                telemetry.addData("TagYaw", "%.1f°", robot.getTagYawDeg());
            }
            telemetry.update();

            // Loop timing (ms): keep small for responsiveness and timing accuracy.
            sleep(20);
        }
    }

    private boolean isShooterReady(double percentTolerance, double minTicksPerSecond) {
        if (robot.isShooterAtSpeedPercent(percentTolerance)) {
            return true;
        }
        if (minTicksPerSecond > 0.0) {
            return Math.abs(robot.getShooterVelocityPerSecond()) >= minTicksPerSecond;
        }
        return false;
    }
}

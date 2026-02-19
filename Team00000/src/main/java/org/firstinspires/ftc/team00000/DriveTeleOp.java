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

/**
 * Driver-controlled OpMode for Team00000.
 *
 * <p>Teaching goals for students:
 * 1) Separate robot services ({@link RobotHardware}) from operator state logic in this class.
 * 2) Demonstrate edge-triggered input handling and timed state machines.
 * 3) Expose key tuning values in FTC Dashboard via {@link Config}.
 *
 * <p>Primary controls:
 * - Drive: left stick translate, right stick rotate, left bumper precision mode
 * - Vision assist: hold right bumper for AprilTag approach
 * - Shooter: X toggles spin-up, Y runs timed burst feed
 * - Shooter tuning: right stick button toggles AUTO_TAG, triggers trim model output
 * - Transfer/stoppers: d-pad and A/B intake/reverse/gate control
 */
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

    // Current mechanism outputs (also visible and editable from Dashboard).
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
    private boolean prevR3 = false;
    private boolean prevTrimInc = false;
    private boolean prevTrimDec = false;
    private boolean prevTrimReset = false;

    // Runtime operator state machine.
    private boolean shooterEnabled = false;
    private boolean intakeMode = false;
    private boolean launchMode = false;
    private long launchStartMs = 0;
    private long readySinceMs = 0;
    private long feedStartMs = 0;
    private long feedAccumulatedMs = 0;
    private long lastLaunchLoopMs = 0;
    private boolean jamReverseActive = false;
    private double jamRestoreTransferPower = 0.0;
    private double jamRestoreStopperPower = 0.0;
    private long lastTrimStepMs = 0;

    // Shot-profile multipliers used within a burst feed sequence.
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

    // Shooter readiness and feed gating.
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

    // AUTO_TAG shooter model parameters (range in inches, command in [0..1]).
    public static boolean SHOOTER_AUTO_TAG = false;
    public static double SHOOTER_AUTO_REF_RANGE_IN = 140.0;
    public static double SHOOTER_AUTO_REF_CMD = 0.38;
    public static double SHOOTER_AUTO_CMD_PER_IN = 0.0018;
    public static double SHOOTER_AUTO_MIN_CMD = 0.20;
    public static double SHOOTER_AUTO_MAX_CMD = 0.90;
    public static double SHOOTER_AUTO_FALLBACK_CMD = 0.38;
    public static double SHOOTER_AUTO_RANGE_ALPHA = 0.25;
    public static int SHOOTER_AUTO_RANGE_HOLD_MS = 500;
    public static double SHOOTER_VOLTAGE_NOMINAL = 12.0;
    public static double SHOOTER_VOLTAGE_COMP_WEIGHT = 1.0;
    public static double SHOOTER_VOLTAGE_COMP_MIN = 0.90;
    public static double SHOOTER_VOLTAGE_COMP_MAX = 1.20;
    public static double SHOOTER_AUTO_TRIM_CMD = 0.0;
    public static double SHOOTER_TRIM_STEP_CMD = 0.005;
    public static double SHOOTER_TRIM_MAX_CMD = 0.20;
    public static double SHOOTER_TRIM_TRIGGER_THRESHOLD = 0.60;
    public static int SHOOTER_TRIM_REPEAT_MS = 120;

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

        // Initialize hardware once, then mirror telemetry to Driver Station + Dashboard.
        robot.init();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);
        SHOOTER_AUTO_TRIM_CMD = Range.clip(SHOOTER_AUTO_TRIM_CMD,
                -Math.abs(SHOOTER_TRIM_MAX_CMD), Math.abs(SHOOTER_TRIM_MAX_CMD));
        lastTrimStepMs = 0;
        robot.resetShooterAutoModel(SHOOTER_AUTO_FALLBACK_CMD);

        while(opModeInInit()) {
            // Pre-start checks for students: heading, mode, and tuning controls.
            telemetry.addData("Status", "Hardware Initialized");
            telemetry.addData("Heading", "%4.0f", robot.getHeading());
            telemetry.addData("ShooterMode", SHOOTER_AUTO_TAG ? "AUTO_TAG" : "MANUAL");
            telemetry.addData("ShooterTrim", "%+.3f", SHOOTER_AUTO_TRIM_CMD);
            telemetry.addData("TuneCtrl", "R3 mode  RT/LT trim  RT+LT reset");
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

            // Keep vision data fresh for assist, telemetry, and shooter auto model.
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
            boolean r3 = gamepad1.right_stick_button;
            boolean trimIncPressed = gamepad1.right_trigger >= SHOOTER_TRIM_TRIGGER_THRESHOLD;
            boolean trimDecPressed = gamepad1.left_trigger >= SHOOTER_TRIM_TRIGGER_THRESHOLD;
            boolean trimResetPressed = trimIncPressed && trimDecPressed;
            long loopNowMs = System.currentTimeMillis();

            boolean upEdge = up && !prevUp;
            boolean downEdge = down && !prevDown;
            boolean leftEdge = left && !prevLeft;
            boolean rightEdge = right && !prevRight;
            boolean aEdge = a && !prevA;
            boolean xEdge = x && !prevX;
            boolean yEdge = y && !prevY;
            boolean l3Edge = l3 && !prevL3;
            boolean r3Edge = r3 && !prevR3;

            // Left stick button toggles drive frame: field-centric vs robot-centric.
            if (l3Edge) {
                driveMode = (driveMode == DriveMode.FIELD) ? DriveMode.ROBOT : DriveMode.FIELD;
                robot.resetYaw();
            }

            // Right stick button toggles auto shooter modeling.
            if (r3Edge) {
                SHOOTER_AUTO_TAG = !SHOOTER_AUTO_TAG;
            }
            updateAutoShooterTrim(loopNowMs, trimIncPressed, trimDecPressed, trimResetPressed);

            // D-pad UP/DOWN directly command transfer direction.
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

            // D-pad LEFT is a hard stop for transfer + stopper.
            if (leftEdge) {
                transferPower = 0.0;
                stopperPower = STOPPER_CLOSED;
                intakeMode = false;
                launchMode = false;
                shotIndex = 0;
            }

            // D-pad RIGHT toggles stopper open/closed.
            if (rightEdge) {
                stopperPower = (stopperPower == STOPPER_OPEN) ? STOPPER_CLOSED : STOPPER_OPEN;
            }

            // A toggles intake mode.
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

            // B (hold) is a temporary anti-jam reverse override.
            if (b) {
                if (!jamReverseActive) {
                    if (launchMode) {
                        jamRestoreTransferPower = 0.0;
                        jamRestoreStopperPower = STOPPER_CLOSED;
                    } else {
                        jamRestoreTransferPower = transferPower;
                        jamRestoreStopperPower = stopperPower;
                    }
                    jamReverseActive = true;
                }
                transferPower = TRANSFER_REV;
                stopperPower = STOPPER_CLOSED;
                launchMode = false;
            } else if (jamReverseActive) {
                jamReverseActive = false;
                transferPower = jamRestoreTransferPower;
                stopperPower = jamRestoreStopperPower;
            }

            // X toggles shooter flywheel spin-up.
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

            // Y starts burst feed sequence (if shooter already enabled).
            if (yEdge && shooterEnabled) {
                launchMode = true;
                intakeMode = false;
                launchStartMs = loopNowMs;
                readySinceMs = 0;
                feedStartMs = 0;
                feedAccumulatedMs = 0;
                lastLaunchLoopMs = launchStartMs;
                firingShotIndex = Range.clip(shotIndex, 0, 2);
                preFeedTicksPerSecond = Double.NaN;
                minimumFeedTicksPerSecond = Double.NaN;
            }

            if(launchMode && shooterEnabled) {
                long now = loopNowMs;
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
                    boolean atSpeed = robot.isShooterReady(SHOOT_READY_PCT_TOL, SHOOT_READY_MIN_TPS);

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
                        // Spool phase: hold feed closed until ready (or timeout policy triggers).
                        stopperPower = STOPPER_CLOSED;
                        transferPower = 0.0;
                        feedStartMs = 0;
                    } else {
                        // Latch feed start once, then accumulate active feed time.
                        if (feedStartMs == 0) {
                            feedStartMs = now;
                            preFeedTicksPerSecond = Math.abs(robot.getShooterVelocityPerSecond());
                            minimumFeedTicksPerSecond = preFeedTicksPerSecond;
                        }

                        long totalFeedMs = (long) LAUNCH_FEED_MS * Math.max(1, LAUNCH_ARTIFACTS_PER_BURST);
                        boolean atFeedSpeed = robot.isShooterReady(SHOOT_FEED_PCT_TOL, SHOOT_FEED_MIN_TPS);
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
                                // Pause feed if speed drops below threshold.
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
                // Safety default outside launch mode.
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
            prevR3 = r3;
            prevTrimInc = trimIncPressed;
            prevTrimDec = trimDecPressed;
            prevTrimReset = trimResetPressed;

            // Slow mode scales driver inputs for precision driving.
            boolean slow = gamepad1.left_bumper;
            double scale = slow ? 0.4 : 1.0;

            axial = -gamepad1.left_stick_y * scale;
            lateral = gamepad1.left_stick_x * scale;
            yaw = gamepad1.right_stick_x * scale;

            // Use motor metadata maxRPM unless Dashboard override is active.
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
            // AprilTag measurements drive both shooter model and operator telemetry.
            Integer goalId = robot.getGoalTagId();
            double range = robot.getGoalRangeIn();
            double bearing = robot.getGoalBearingDeg();
            double elevation = robot.getGoalElevationDeg();

            SHOOTER_AUTO_TRIM_CMD = Range.clip(SHOOTER_AUTO_TRIM_CMD,
                    -Math.abs(SHOOTER_TRIM_MAX_CMD), Math.abs(SHOOTER_TRIM_MAX_CMD));
            robot.updateShooterAutoModel(
                    range,
                    loopNowMs,
                    SHOOTER_AUTO_RANGE_ALPHA,
                    SHOOTER_AUTO_RANGE_HOLD_MS,
                    SHOOTER_AUTO_REF_RANGE_IN,
                    SHOOTER_AUTO_REF_CMD,
                    SHOOTER_AUTO_CMD_PER_IN,
                    SHOOTER_AUTO_FALLBACK_CMD,
                    SHOOTER_AUTO_MIN_CMD,
                    SHOOTER_AUTO_MAX_CMD,
                    SHOOTER_VOLTAGE_NOMINAL,
                    SHOOTER_VOLTAGE_COMP_WEIGHT,
                    SHOOTER_VOLTAGE_COMP_MIN,
                    SHOOTER_VOLTAGE_COMP_MAX,
                    SHOOTER_AUTO_TRIM_CMD);

            double shooterBase = 0.0;
            if (shooterEnabled) {
                shooterBase = SHOOTER_AUTO_TAG ? robot.getShooterAutoBaseCmd() : shooterFar;
            }

            int effectiveShotIndex;
            if (launchMode && shooterEnabled && feedStartMs != 0) {
                effectiveShotIndex = firingShotIndex + getBurstStepFromAccumulatedFeedMs();
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

            // Simple derived helpers for students while tuning.
            double horizontal = (Double.isNaN(range) || Double.isNaN(bearing))
                    ? Double.NaN
                    : range * Math.cos(Math.toRadians(bearing));
            double aimAboveHorizontal = (Double.isNaN(elevation) ? Double.NaN : (8.0 + elevation));

            // RB (hold): AprilTag drive assist (range->axial, tagYaw->lateral, bearing->turn).
            boolean autoAssist = gamepad1.right_bumper;
            boolean didAuto = false;
            if (autoAssist) {
                // Refresh again right before assist step for best recency.
                robot.updateAprilTagDetections();
                didAuto = robot.autoDriveToGoalStep();
            }

            // Manual drive fallback when assist is inactive or has no valid target.
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
                activeShotIndex = firingShotIndex + getBurstStepFromAccumulatedFeedMs();
            } else {
                activeShotIndex = shotIndex;
            }
            activeShotIndex = Range.clip(activeShotIndex, 0, 2);

            telemetry.addData("Shooter", shooterEnabled ? "ON" : "OFF");
            telemetry.addData("ShooterMode", SHOOTER_AUTO_TAG ? "AUTO_TAG" : "MANUAL");
            telemetry.addData("Launch", launchMode ? "ON" : "OFF");
            telemetry.addData("ShooterTPS", "%.0f", currentTicksPerSecond);
            telemetry.addData("ShooterCmd", "%.3f", shooterApplied);
            telemetry.addData("Trim", "%+.3f", SHOOTER_AUTO_TRIM_CMD);
            telemetry.addData("Shot", "%d", (activeShotIndex + 1));
            telemetry.addData("AtSpeed", "%s",
                    robot.isShooterReady(SHOOT_READY_PCT_TOL, SHOOT_READY_MIN_TPS) ? "YES" : "no");
            telemetry.addData("FeedReady", "%s",
                    robot.isShooterReady(SHOOT_FEED_PCT_TOL, SHOOT_FEED_MIN_TPS) ? "YES" : "no");

            if (TELEMETRY_VERBOSE) {
                double filteredRangeIn = robot.getShooterAutoFilteredRangeIn();
                double batteryVoltage = robot.getShooterAutoBatteryVoltage();
                String filteredRangeText = Double.isNaN(filteredRangeIn)
                        ? "–"
                        : String.format("%.1f", filteredRangeIn);
                String batteryText = Double.isNaN(batteryVoltage)
                        ? "–"
                        : String.format("%.2fV", batteryVoltage);
                telemetry.addData("Assist", autoAssist ? (didAuto ? "AUTO→TAG" : "NO TAG") : "MANUAL");
                telemetry.addData("DriveRaw", "ax=%.2f  lat=%.2f  yaw=%.2f", axial, lateral, yaw);
                telemetry.addData("ShooterCommand", "%.2f", shooterBase);
                telemetry.addData("ShooterCmdApplied", "%.2f", shooterApplied);
                telemetry.addData("ShooterAuto", "%s trim=%+.3f", SHOOTER_AUTO_TAG ? "ON" : "OFF", SHOOTER_AUTO_TRIM_CMD);
                telemetry.addData("ShooterAutoBase", "%.3f  voltComp=%.3f", robot.getShooterAutoBaseCmd(), robot.getShooterAutoVoltageComp());
                telemetry.addData("ShooterAutoRange", "raw=%.1f in  filt=%s in", range, filteredRangeText);
                telemetry.addData("Battery", "%s", batteryText);
                telemetry.addData("TuneCtrl", "R3=AUTO_TAG  RT/LT=trim  RT+LT=reset");
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

            // Fixed loop delay keeps control timing predictable and CPU load reasonable.
            sleep(20);
        }
    }

    /**
     * Converts accumulated active feed time into burst profile step index.
     *
     * <p>Using accumulated feed time (instead of wall time) keeps shot progression stable when feed
     * pauses for speed recovery.
     */
    private int getBurstStepFromAccumulatedFeedMs() {
        if (LAUNCH_FEED_MS <= 0) {
            return 0;
        }
        return Range.clip((int) (feedAccumulatedMs / (long) LAUNCH_FEED_MS), 0, 2);
    }

    /**
     * Applies live operator trim adjustments to AUTO_TAG shooter command.
     *
     * <p>Controls:
     * RT increases trim, LT decreases trim, RT+LT resets trim.
     */
    private void updateAutoShooterTrim(long nowMs, boolean trimIncPressed, boolean trimDecPressed, boolean trimResetPressed) {
        SHOOTER_AUTO_TRIM_CMD = Range.clip(SHOOTER_AUTO_TRIM_CMD,
                -Math.abs(SHOOTER_TRIM_MAX_CMD), Math.abs(SHOOTER_TRIM_MAX_CMD));
        if (!SHOOTER_AUTO_TAG) {
            lastTrimStepMs = 0;
            return;
        }
        if (trimResetPressed) {
            if (!prevTrimReset) {
                SHOOTER_AUTO_TRIM_CMD = 0.0;
                lastTrimStepMs = nowMs;
            }
            return;
        }

        boolean singleDirection = trimIncPressed ^ trimDecPressed;
        if (!singleDirection) {
            return;
        }

        boolean risingEdge = (trimIncPressed && !prevTrimInc) || (trimDecPressed && !prevTrimDec);
        boolean repeatReady = (lastTrimStepMs == 0)
                || (nowMs - lastTrimStepMs >= Math.max(20, SHOOTER_TRIM_REPEAT_MS));
        if (risingEdge || repeatReady) {
            double step = trimIncPressed ? Math.abs(SHOOTER_TRIM_STEP_CMD) : -Math.abs(SHOOTER_TRIM_STEP_CMD);
            SHOOTER_AUTO_TRIM_CMD = Range.clip(SHOOTER_AUTO_TRIM_CMD + step,
                    -Math.abs(SHOOTER_TRIM_MAX_CMD), Math.abs(SHOOTER_TRIM_MAX_CMD));
            lastTrimStepMs = nowMs;
        }
    }

}

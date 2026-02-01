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

// Unified TeleOp with a drive-mode toggle.
// Drive: LS=translate, RS=rotate, LB=slow mode.
// Assist: hold RB for AprilTag assist.
// Shooter: X=spin-up toggle, Y=burst feed.
// Transfer: D-pad UP/DOWN sticky fwd/rev, LEFT safe stop, RIGHT stopper toggle, A intake toggle, B hold quick reverse.
// Toggle drive mode (FIELD/ROBOT): press LS (left stick button).
@Config
@TeleOp(name = "Drive TeleOp", group = "opMode")

public class DriveTeleOp extends LinearOpMode {

    // Hardware abstraction (motors, IMU, AprilTag vision).
    RobotHardware robot = new RobotHardware(this);

    // Shooter tuning (Dashboard-configurable).
    public static double shooter = 0.475;
    public static double shooterMaxRpm = 6000.0;
    public static double shooterKp = 0.0;
    public static double shooterKi = 0.0;
    public static double shooterKd = 0.0;

    // Mechanism outputs (Dashboard-visible for tuning).
    public static double stopperPower = 0.0;
    public static double transferPower = 0.0;

    // Edge-detect state for buttons.
    private boolean prevUp = false;
    private boolean prevDown = false;
    private boolean prevLeft = false;
    private boolean prevRight = false;
    private boolean prevA = false;
    private boolean prevX = false;
    private boolean prevY = false;
    private boolean prevLsBtn = false;

    // Driver modes/state.
    private boolean shooterEnabled = false;
    private boolean intakeMode = false;
    private boolean launchMode = false;
    private long launchStartMs = 0;

    // Drive mode selection (field-centric vs robot-centric).
    private enum DriveMode { FIELD, ROBOT }
    private DriveMode driveMode = DriveMode.FIELD;

    // Burst timing (milliseconds).
    public static int LAUNCH_SPOOL_MS = 0;
    public static int LAUNCH_FEED_MS = 1200;

    // Mechanism power presets.
    public static double TRANSFER_FWD = 0.80;
    public static double TRANSFER_REV = -0.20;
    public static double STOPPER_OPEN = 0.80;
    public static double STOPPER_CLOSED = 0.00;

    @Override
    public void runOpMode() {

        double axial;
        double lateral;
        double yaw;

        // Initialize hardware and dashboard telemetry.
        robot.init();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);

        while(opModeInInit()) {
            // Pre-start checks: verify heading updates and vision is initialized.
            telemetry.addData("Status", "Hardware Initialized");
            telemetry.addData("Vision", "Ready (AprilTag)");
            telemetry.addData("Mode", "INIT");
            telemetry.addData("Obelisk", robot.hasObeliskMotif() ? String.format("%s (ID %s)",
                    robot.getObeliskMotif(), robot.getObeliskTagId()) : "–");
            telemetry.addData("Heading", "%4.0f", robot.getHeading());
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // Shooter telemetry smoothing (exponential moving average).
        double shooterActL_f = 0.0;
        double shooterActR_f = 0.0;
        double shooterRpmL_f = 0.0;
        double shooterRpmR_f = 0.0;
        final double shooterAlpha = 0.20; // Higher = more responsive, lower = smoother.

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
            boolean lsBtn = gamepad1.left_stick_button;

            boolean upEdge = up && !prevUp;
            boolean downEdge = down && !prevDown;
            boolean leftEdge = left && !prevLeft;
            boolean rightEdge = right && !prevRight;
            boolean aEdge = a && !prevA;
            boolean xEdge = x && !prevX;
            boolean yEdge = y && !prevY;
            boolean lsBtnEdge = lsBtn && !prevLsBtn;

            if (lsBtnEdge) {
                driveMode = (driveMode == DriveMode.FIELD) ? DriveMode.ROBOT : DriveMode.FIELD;
            }

            // D-pad UP/DOWN: transfer control (edge-triggered; no hold required).
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

            // D-pad LEFT: safe stop (transfer off, stoppers closed).
            if (leftEdge) {
                transferPower = 0.0;
                stopperPower = STOPPER_CLOSED;
                intakeMode = false;
                launchMode = false;
            }

            // D-pad RIGHT: toggle stoppers (manual override).
            if (rightEdge) {
                stopperPower = (stopperPower == STOPPER_OPEN) ? STOPPER_CLOSED : STOPPER_OPEN;
            }

            // A: toggle intake mode (runs transfer forward; stoppers forced closed).
            if (aEdge) {
                intakeMode = !intakeMode;
                launchMode = false;
                if (intakeMode) {
                    transferPower = TRANSFER_FWD;
                } else {
                    transferPower = 0.0;
                }
                stopperPower = STOPPER_CLOSED;
            }

            // B (hold): quick reverse to clear jams (overrides other modes).
            if (b) {
                transferPower = TRANSFER_REV;
                intakeMode = false;
                launchMode = false;
            }

            // X: toggle shooter spin-up (no feeding).
            if (xEdge) {
                shooterEnabled = !shooterEnabled;
                if (!shooterEnabled) {
                    launchMode = false;
                    stopperPower = STOPPER_CLOSED;
                    transferPower = 0.0;
                    intakeMode = false;
                }
            }

            // Y: burst feed for a fixed time window; requires shooter enabled.
            if (yEdge && shooterEnabled) {
                launchMode = true;
                intakeMode = false;
                launchStartMs = System.currentTimeMillis();
            }

            if(launchMode && shooterEnabled) {
                long t = System.currentTimeMillis() - launchStartMs;

                if (t < LAUNCH_SPOOL_MS) {
                    // Spool phase: keep stoppers closed and do not feed.
                    stopperPower = STOPPER_CLOSED;
                    transferPower = 0.0;
                } else if (t < (LAUNCH_SPOOL_MS + LAUNCH_FEED_MS)) {
                    stopperPower = STOPPER_OPEN;
                    transferPower = TRANSFER_FWD;
                } else {
                    stopperPower = STOPPER_CLOSED;
                    transferPower = 0.0;
                    launchMode = false;
                }
            } else {
                // Safety: keep stoppers closed while intaking.
                if (intakeMode && !b) {
                    stopperPower = STOPPER_CLOSED;
                }
            }

            prevUp = up;
            prevDown = down;
            prevLeft = left;
            prevRight = right;
            prevA = a;
            prevX = x;
            prevY = y;
            prevLsBtn = lsBtn;

            // Slow mode scales driver inputs for precision.
            boolean slow = gamepad1.left_bumper;
            double scale = slow ? 0.4 : 1.0;

            axial = -gamepad1.left_stick_y * scale;
            lateral = gamepad1.left_stick_x * scale;
            yaw = gamepad1.right_stick_x * scale;

            // Vision values (for telemetry and driver assist).
            Integer goalId = robot.getGoalTagId();
            double range = robot.getGoalRangeIn();
            double bearing = robot.getGoalBearingDeg();
            double elevation = robot.getGoalElevationDeg();

            // Derived aim helpers (assumes camera pitched up 8°).
            double horiz = (Double.isNaN(range) || Double.isNaN(bearing))
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

            robot.configureShooterVelocityPidfForMaxRpm(shooterMaxRpm, shooterKp, shooterKi, shooterKd);
            double shooterCmb = shooterEnabled ? shooter : 0.0;
            robot.setShooterVelocityPercent(shooterCmb, shooterMaxRpm);
            robot.setStopperPower(stopperPower);
            robot.setTransferPower(transferPower);

            telemetry.addData("Mode", slow ? "SLOW" : "NORMAL");
            telemetry.addData("DriveMode", driveMode == DriveMode.FIELD ? "FIELD" : "ROBOT");
            telemetry.addData("Assist", autoAssist ? (didAuto ? "AUTO→TAG" : "NO TAG") : "MANUAL");
            telemetry.addData("Heading", "%4.0f°", robot.getHeading());
            telemetry.addData("Drive", "ax=%.2f  lat=%.2f  yaw=%.2f", axial, lateral, yaw);
            telemetry.addData("ShooterCmd", "%.2f", shooterEnabled ? shooter : 0.0);
            telemetry.addData("StopperPwr", "%.2f", stopperPower);
            telemetry.addData("TransferPwr", "%.2f", transferPower);
            telemetry.addData("Op", "shooter=%s intake=%s launch=%s",
                    shooterEnabled ? "ON" : "OFF",
                    intakeMode ? "ON" : "OFF",
                    launchMode ? "ON" : "OFF");
            telemetry.addData("ShooterTgt", "%.0f t/s", robot.getShooterTargetTicksPerSec());

            double shooterActL = robot.getTopShooterTicksPerSec();
            double shooterActR = robot.getBottomShooterTicksPerSec();
            double shooterRpmL = robot.getTopShooterRpm();
            double shooterRpmR = robot.getBottomShooterRpm();

            shooterActL_f = shooterAlpha * shooterActL + (1.0 - shooterAlpha) * shooterActL_f;
            shooterActR_f = shooterAlpha * shooterActR + (1.0 - shooterAlpha) * shooterActR_f;
            shooterRpmL_f = shooterAlpha * shooterRpmL + (1.0 - shooterAlpha) * shooterRpmL_f;
            shooterRpmR_f = shooterAlpha * shooterRpmR + (1.0 - shooterAlpha) * shooterRpmR_f;

            telemetry.addData("ShooterAct", "L=%.0f  R=%.0f t/s", shooterActL_f, shooterActR_f);
            telemetry.addData("ShooterRPM", "L=%.0f  R=%.0f", shooterRpmL_f, shooterRpmR_f);

            String motif = robot.hasObeliskMotif() ? String.format("%s (ID %s)", robot.getObeliskMotif(), robot.getObeliskTagId()) : "–";
            telemetry.addData("Obelisk", motif);

            telemetry.addData("Goal", (goalId != null) ? goalId : "–");
            telemetry.addData("Pose", "rng=%.1f in  brg=%.1f°  elev=%.1f°", range, bearing, elevation);
            telemetry.addData("Aim",  "horiz=%.1f in  aboveHoriz=%s",
                    horiz,
                    Double.isNaN(aimAboveHorizontal) ? "–" : String.format("%.1f°", aimAboveHorizontal));
            telemetry.addData("TagYaw", "%.1f°", robot.getTagYawDeg());
            telemetry.update();

            // Loop timing (ms): keep small for responsiveness and timing accuracy.
            sleep(20);
        }
    }
}
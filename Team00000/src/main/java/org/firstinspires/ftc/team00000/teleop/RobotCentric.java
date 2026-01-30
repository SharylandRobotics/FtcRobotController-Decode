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

package org.firstinspires.ftc.team00000.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team00000.RobotHardware;

import static org.firstinspires.ftc.team00000.teleop.FieldCentric.*;

@TeleOp(name = "Robot Centric", group = "opMode")

public class RobotCentric extends LinearOpMode {

    // Create hardware instance and pass current OpMode reference
    RobotHardware robot = new RobotHardware(this);
    public static double shooter = 0.4;

    public static double stopperPower = 0.0;
    public static double transferPower = 0.0;

    private boolean prevUp = false;
    private boolean prevDown = false;
    private boolean prevLeft = false;
    private boolean prevRight = false;
    private boolean prevA = false;
    private boolean prevX = false;
    private boolean prevY = false;

    private boolean shooterEnabled = false;
    private boolean intakeMode = false;
    private boolean launchMode = false;
    private long launchStartMs = 0;

    public static int LAUNCH_SPOOL_MS = 300;
    public static int LAUNCH_FEED_MS = 6000;

    public static double TRANSFER_FWD = 0.80;
    public static double TRANSFER_REV = -0.20;
    public static double STOPPER_OPEN = 0.80;
    public static double STOPPER_CLOSED = 0.00;

    @Override
    public void runOpMode() {

        // Variables for joystick input: forward/back (axial), strafe (lateral), rotation (yaw)
        double axial;
        double lateral;
        double yaw;

        // Initialize all motors, IMU, and hardware configuration
        robot.init();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);

        while(opModeInInit()) {
            // Student Note: Pre‑start check — rotate robot by hand; heading should change.
            // "Vision: Ready (AprilTag)" means camera + processor initialized.
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

        // Shooter telemetry smoothing (exponential moving average)
        double shooterActL_f = 0.0;
        double shooterActR_f = 0.0;
        double shooterRpmL_f = 0.0;
        double shooterRpmR_f = 0.0;
        final double shooterAlpha = 0.20; // higher = more responsive, lower = smoother

        // Main control loop: continuously while TeleOp is active
        while (opModeIsActive()) {

            // Keep vision fresh before using pose values each loop
            robot.updateAprilTagDetections();

            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            boolean left = gamepad1.dpad_left;
            boolean right = gamepad1.dpad_right;
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;
            boolean y = gamepad1.y;

            boolean upEdge = up && !prevUp;
            boolean downEdge = down && !prevDown;
            boolean leftEdge = left && !prevLeft;
            boolean rightEdge = right && !prevRight;
            boolean aEdge = a && !prevA;
            boolean xEdge = x && !prevX;
            boolean yEdge = y && !prevY;

            // Dpad UP/DOWN = set transfer forward/reverse (edge triggered)
            // No hold required: stays at last commanded power.
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

            // Dpad LEFT = hard stop transfer + close stoppers (safe)
            if (leftEdge) {
                transferPower = 0.0;
                stopperPower = STOPPER_CLOSED;
                intakeMode = false;
                launchMode = false;
            }

            // Dpad RIGHT = manual stopper toggle (testing)
            if (rightEdge) {
                stopperPower = (stopperPower == STOPPER_OPEN) ? STOPPER_CLOSED : STOPPER_OPEN;
            }

            // A = toggle intake mode (transfer runs forward; stoppers forced closed)
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

            // B = momentary clear: reverse transfer while held (overrides intake/launch)
            if (b) {
                transferPower = TRANSFER_REV;
                intakeMode = false;
                launchMode = false;
            }

            // X = toggle shooter enable (spins up but does not feed)
            if (xEdge) {
                shooterEnabled = !shooterEnabled;
                if (!shooterEnabled) {
                    launchMode = false;
                    stopperPower = STOPPER_CLOSED;
                    transferPower = 0.0;
                    intakeMode = false;
                }
            }

            // Y = fire one burst (edge triggered): spool shooter, then open stoppers + feed briefly
            if (yEdge && shooterEnabled) {
                launchMode = true;
                intakeMode = false;
                launchStartMs = System.currentTimeMillis();
            }

            if(launchMode && shooterEnabled) {
                long t = System.currentTimeMillis() - launchStartMs;

                if (t < LAUNCH_SPOOL_MS) {
                    // Spool only: keeps stoppers closed, no feeding
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
                // Safety: while intaking, keep stoppers closed so transfer can't feed into shooter
                if (intakeMode && !b) {
                    stopperPower = STOPPER_CLOSED;
                }
            }

            // Save previous button states
            prevUp = up;
            prevDown = down;
            prevLeft = left;
            prevRight = right;
            prevA = a;
            prevX = x;
            prevY = y;

            boolean slow = gamepad1.left_bumper;
            double scale = slow ? 0.4 : 1.0;

            // Read real-time joystick values from gamepad
            axial = -gamepad1.left_stick_y * scale;
            lateral = gamepad1.left_stick_x * scale;
            yaw = gamepad1.right_stick_x * scale;

            // --- Vision helpers for concise telemetry ---
            Integer goalId = robot.getGoalTagId();
            double range = robot.getGoalRangeIn();
            double bearing = robot.getGoalBearingDeg();
            double elevation = robot.getGoalElevationDeg();

            // Approx horizontal distance and aim-above-horizontal (camera pitched up 15°)
            double horiz = (Double.isNaN(range) || Double.isNaN(bearing))
                    ? Double.NaN
                    : range * Math.cos(Math.toRadians(bearing));
            double aimAboveHorizontal = (Double.isNaN(elevation) ? Double.NaN : (15.0 + elevation));

            // Driver Assist: hold RB to auto-drive toward the visible goal tag (range->drive, yaw->strafe, bearing->turn).
            boolean autoAssist = gamepad1.right_bumper;
            boolean didAuto = false;
            if (autoAssist) {
                robot.updateAprilTagDetections();
                didAuto = robot.autoDriveToGoalStep();
            }

            // Student Note: Field‑centric drive call (mixing happens in RobotHardware) unless auto applied.
            if (!didAuto) {
                // Apply joystick inputs directly to robot-centric drive control
                robot.driveRobotCentric(axial, lateral, yaw);
            }

            robot.configureShooterVelocityPidfForMaxRpm(shooterMaxRpm, shooterKp, shooterKi, shooterKd);
            double shooterCmb = shooterEnabled ? shooter : 0.0;
            robot.setShooterVelocityPercent(shooterCmb, shooterMaxRpm);
            robot.setStopperPower(stopperPower);
            robot.setTransferPower(transferPower);

            telemetry.addData("Mode", slow ? "SLOW" : "NORMAL");
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

            // Small delay to prevent telemetry flooding
            sleep(50);
        }
    }
}


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

package org.firstinspires.ftc.team13580.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team13580.RobotHardware;

// Student Notes: Field‑centric TeleOp. Left stick = drive/strafe, Right stick = turn, LB = slow mode.
// TODO(students): Adjust slow‑mode scale if you want finer aiming.
@TeleOp(name = "Field Centric", group = "opMode")

public class FieldCentric extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        double kickerBackPos = 0;
        double kickerForwardPos = 1;
        double kickerleftBackPos = 0;
        double kickerleftFowardPos =1;

        double axial;
        double lateral;
        double yaw;

        double intake;
        double outtake;

        // Student Note: Initialize hardware (motors, IMU, vision).
        robot.init();

        while(opModeInInit()) {
            // Student Note: Pre‑start check — rotate robot by hand; heading should change.
            // "Vision: Ready (AprilTag)" means camera + processor initialized.
            telemetry.addData("Status", "Hardware Initialized");
            telemetry.addData("Vision", "Ready (AprilTag)");
            telemetry.addData("Mode", "INIT");
            //telemetry.addData("Obelisk", robot.hasObeliskMotif() ? String.format("%s (ID %s)",
                    //robot.getObeliskMotif(), robot.getObeliskTagId()) : "–");
            telemetry.addData("Heading", "%4.0f", robot.getHeading());
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Keep vision fresh before using pose values each loop
            robot.updateAprilTagDetections();

            // Student Note: Hold LB for precision (slow) mode.
            boolean slow = gamepad1.left_bumper;
            double scale = slow ? 0.4 : 1.0;

            axial = -gamepad1.left_stick_y * scale;
            lateral = gamepad1.left_stick_x * scale;
            yaw = gamepad1.right_stick_x * scale;

            // --- Vision helpers for concise telemetry ---
            //Integer goalId = robot.getGoalTagId();
            //double range = robot.getGoalRangeIn();
            //double bearing = robot.getGoalBearingDeg();
            //double elevation = robot.getGoalElevationDeg();

            // Approx horizontal distance and aim-above-horizontal (camera pitched up 15°)
            //double horiz = (Double.isNaN(range) || Double.isNaN(bearing))
            //        ? Double.NaN
            //        : range * Math.cos(Math.toRadians(bearing));
            //double aimAboveHorizontal = (Double.isNaN(elevation) ? Double.NaN : (15.0 + elevation));

            // Driver Assist: hold RB to auto-drive toward the visible goal tag (range->drive, yaw->strafe, bearing->turn).
            robot.updateAprilTagDetections();
            boolean didAuto = robot.autoDriveToGoalStep();

            // Student Note: Field‑centric drive call (mixing happens in RobotHardware) unless auto applied.
            if (!didAuto) {
                robot.driveFieldCentric(axial, lateral, yaw);
            }

            if (gamepad1.right_trigger == 1) {
                intake = .6;
            } else if (gamepad1.dpad_down) {
                intake = -1;
            } else if (gamepad1.right_bumper) {
                intake = .3;
            } else {
                intake = 0;
            }

            outtake = 0;
            if (didAuto) {
                // autoDriveToGoalStep() already sets outtake speed
            } else if (gamepad2.y) {
                outtake = 1300;
            } else {
                outtake = gamepad1.left_trigger*2000;
            }
            robot.setOuttakeVelocity((int) outtake);


            if (gamepad1.b) {
                robot.setKickerPower(kickerForwardPos);
            } else {
                robot.setKickerPower(kickerBackPos);
            }

            if (gamepad1.a) {
                robot.setKickerLeftPower(kickerleftFowardPos);
            } else {
                robot.setKickerLeftPower(kickerleftBackPos);
            }

            robot.setIntakePower(intake);
            robot.setOuttakeVelocity((int) outtake);

            telemetry.addData("Mode", slow ? "SLOW" : "NORMAL");
            telemetry.addData("Assist", didAuto ? "AUTO→TAG" : "MANUAL");
            telemetry.addData("Heading", "%4.0f°", robot.getHeading());
            telemetry.addData("Drive", "ax=%.2f  lat=%.2f  yaw=%.2f", axial, lateral, yaw);

            //String motif = robot.hasObeliskMotif() ? String.format("%s (ID %s)", robot.getObeliskMotif(), robot.getObeliskTagId()) : "–";
            //telemetry.addData("Obelisk", motif);

            //telemetry.addData("Goal", (goalId != null) ? goalId : "–");
            //telemetry.addData("Pose", "rng=%.1f in  brg=%.1f°  elev=%.1f°", range, bearing, elevation);
            //telemetry.addData("Aim",  "horiz=%.1f in  aboveHoriz=%s",
            //        horiz,
            //        Double.isNaN(aimAboveHorizontal) ? "–" : String.format("%.1f°", aimAboveHorizontal));
            //telemetry.addData("TagYaw", "%.1f°", robot.getTagYawDeg());
            telemetry.update();

            sleep(50);
        }
    }
}


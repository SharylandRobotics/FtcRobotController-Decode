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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team00000.RobotHardware;

// Student Notes: Field‑centric TeleOp. Left stick = drive/strafe, Right stick = turn, LB = slow mode.
// TODO(students): Adjust slow‑mode scale if you want finer aiming.
@TeleOp(name = "Field Centric", group = "opMode")

public class FieldCentric extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {

        double axial;
        double lateral;
        double yaw;

        // Student Note: Initialize hardware (motors, IMU, vision).
        robot.init();

        while(opModeInInit()) {
            // Student Note: Pre‑start check — rotate robot by hand; heading should change.
            // "Vision: Ready (AprilTag)" means camera + processor initialized.
            telemetry.addData("Status", "Hardware Initialized");
            telemetry.addData("Vision", "Ready (AprilTag)");
            telemetry.addData("Obelisk", robot.hasObeliskMotif() ? String.format("%s (ID %s)",
                    robot.getObeliskMotif(), robot.getObeliskTagId()) : "–");
            telemetry.addData("Heading", "%4.0f", robot.getHeading());
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Student Note: Hold LB for precision (slow) mode.
            boolean slow = gamepad1.left_bumper;
            double scale = slow ? 0.4 : 1.0;

            axial = -gamepad1.left_stick_y * scale;
            lateral = gamepad1.left_stick_x * scale;
            yaw = gamepad1.right_stick_x * scale;

            // Student Note: Field‑centric drive call (mixing happens in RobotHardware).
            robot.driveFieldCentric(axial, lateral, yaw);

            telemetry.addData("Controls", "Drive/Strafe: Left Stick | Turn: Right Stick");
            telemetry.addData("Mode", slow ? "SLOW" : "NORMAL");
            telemetry.addData("Heading", "%4.0f°", robot.getHeading());
            telemetry.addData("Inputs", "ax=%.2f   lat=%.2f   yaw=%.2f", axial, lateral, yaw);

            String motif = robot.hasObeliskMotif () ? String.format("%s (ID %s)", robot.getObeliskMotif(),
                    robot.getObeliskTagId()) : "–";
            telemetry.addData("Obelisk Motif", motif);

            // Student Note: Vision summary — if Goal Tag is "–", aim at a goal tag 1–3 m away and avoid glare.
            // Bearing ~0° when centered; range decreases as you move closer.
            Integer goalId = robot.getGoalTagId();
            double range = robot.getGoalRangeIn();
            double bearing = robot.getGoalBearingDeg();
            double elevation = robot.getGoalElevationDeg();
            telemetry.addData("Goal Tag", (goalId != null) ? goalId : "–");
            telemetry.addData("Goal Pose", "range=%.1f in  bearing=%.1f°  elev=%.1f°", range, bearing, elevation);

            telemetry.update();

            sleep(50);
        }
    }
}


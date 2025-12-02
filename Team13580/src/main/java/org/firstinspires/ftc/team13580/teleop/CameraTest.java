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
@TeleOp(name = "Camera Test", group = "opMode")

public class CameraTest extends LinearOpMode {

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
        double outtake = 0;

        // Student Note: Initialize hardware (motors, IMU, vision).
        robot.init();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            robot.updateAprilTagDetections();

            axial = -gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;

           robot.driveFieldCentric(axial, lateral, yaw);

            if (gamepad1.right_trigger == 1) {
                intake = .6;
            } else if (gamepad1.dpad_down) {
                intake = -1;
            } else if (gamepad1.right_bumper) {
                intake = .3;
            } else {
                intake = 0;
            }

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

            if (gamepad2.y) {
                outtake = 1300;
            } else if (gamepad2.b){
                outtake = 0;
            }

            if (gamepad2.dpad_up){
                outtake += 10;
            } else if (gamepad2.dpad_down){
                outtake -= 10;
            }

            robot.setIntakePower(intake);
            robot.setOuttakeVelocity((int) outtake);

           telemetry.addData("flat distance, in: ", Math.sqrt( Math.pow(robot.getGoalRangeIn(), 2) - Math.pow(16, 2) ) );
           telemetry.addData(" current velocity: ", robot.getOuttakeVelocity());
           telemetry.addData("  target velocity: ", outtake);

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


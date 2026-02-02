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

package org.firstinspires.ftc.team12395.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team12395.RobotHardware;
import org.firstinspires.ftc.team12395.rr.Localizer;

@TeleOp(name="Motor Test", group="TeleOp")
@Config
// TODO(STUDENTS): You may rename this for your robot (e.g., "Field Centric - Comp Bot)
public class MotorDebug extends LinearOpMode {

    // NOTE: One hardware instance per OpMode keeps mapping/IMU use simple and testable
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {

        // Driver inputs (range roughly [-1, 1])
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init();

        Localizer localizer = robot.standardDrive.localizer;

        waitForStart();


        // --- TELEOP LOOP ---
        while (opModeIsActive()) {

            if (gamepad1.x){
                robot.frontLeftDrive.setPower(1);
            } else {
                robot.frontLeftDrive.setPower(0);
            }
            if (gamepad1.y){
                robot.frontRightDrive.setPower(1);
            } else {
                robot.frontRightDrive.setPower(0);
            }

            if (gamepad1.a){
                robot.backLeftDrive.setPower(1);
            } else {
                robot.backLeftDrive.setPower(0);
            }
            if (gamepad1.b){
                robot.backRightDrive.setPower(1);
            } else {
                robot.backRightDrive.setPower(0);
            }

            if (gamepad1.left_bumper){
                robot.spindexer.setPower(0.4);
            } else {
                robot.spindexer.setPower(0);
            }

            if (gamepad1.right_bumper){
                robot.intake.setPower(1);
            } else {
                robot.spindexer.setPower(0);
            }

            if (gamepad1.left_trigger > 0){
                robot.shooter.setPower(1);
            }

            robot.standardDrive.updatePoseEstimate();

            telemetry.addData("Pose:", localizer.toString());
            telemetry.addData("PP heading: ", robot.getPinPointHeading());
            telemetry.addData("Standard heading: ", robot.getHeading());
            telemetry.addData("U spindexer: ", robot.getCurrentSpindexerDegreesPos());
            telemetry.addData("E spindexer: ", robot.spindexerE.getPositionAndVelocity().position/robot.spindexerETicksPerDegree);
            telemetry.update();

            sleep(50);
        }
    }
}


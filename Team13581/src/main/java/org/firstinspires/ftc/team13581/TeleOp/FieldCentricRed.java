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

package org.firstinspires.ftc.team13581.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team13581.RobotHardware;

@Config
@TeleOp(name="Field Centric Red Team", group="TeleOp")

public class FieldCentricRed extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    public static double powerTFar = 0.58;
    public static double powerTNear = 0.5;

    @Override
    public void runOpMode() {

        double axial    = 0;
        double lateral  = 0;
        double yaw      = 0;

        boolean runIntake = false;
        boolean runIntakeBackwards = false;

        double velocity = 0;

        robot.init();

        waitForStart();

        while (opModeIsActive()) {

            lateral = -gamepad1.left_stick_y;
            axial = -gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;

            robot.teleOpFieldCentric(axial, lateral, yaw);

            telemetry.addData("Controls", "Drive/Strafe: Left Stick | Turn: Right Stick");
            telemetry.addData("Inputs", "axial=%.2f   lateral=%.2f   yaw=%.2f", axial, lateral, yaw);
            telemetry.addData("Servo Angle: ", robot.getAimPos());
            telemetry.addData("Turret Angle: ", robot.hAimRPos());
            telemetry.addData("Power: ", robot.getBackPower());
            telemetry.addData("Vel: ", robot.getOuttakeRVel());

            telemetry.update();
            if (gamepad1.aWasPressed()){
                runIntake = !runIntake;
                if (runIntake){
                    robot.setIntake1(1);
                    robot.setIntake2(1);
                }
            }
            if (gamepad1.yWasPressed()){
                runIntake = !runIntake;
                if (runIntake){
                    robot.setIntake1(-1);
                    robot.setIntake2(-1);
                }
            }
            if (!runIntake){
                robot.setIntake1(0);
                robot.setIntake2(0);
            }


            if (gamepad1.right_bumper) {
                robot.setIntake2(0);
            }

            if (gamepad2.x) {
                velocity = -700;
            }
            if (gamepad2.a) {
                velocity = 1300;
            }
            if (gamepad2.b) {
                velocity = 0;
            }

            robot.setShootSpeed(velocity);


            if (gamepad2.dpad_down) {
                double tempPos = robot.getAimPos();
                if (tempPos > 0.2) {
                    robot.setAimPos(tempPos - 0.05);
                }
            }

            if (gamepad2.dpad_left) {
                robot.setAimPos(0.45);
            }

            if (gamepad2.dpad_up) {
                double tempPos = robot.getAimPos();
                if (tempPos < (0.9)) {
                    robot.setAimPos(tempPos + 0.05);
                }
            }

            if (gamepad2.left_trigger > 0) {
                double tempPosR = robot.hAimRPos();
                double tempPosL = robot.hAimLPos();
                if (tempPosR > (0)) {
                    robot.setTurretPos(tempPosR - 0.1, tempPosR - 0.1);
                }
            }
            if (gamepad2.right_trigger > 0) {
                double tempPosR = robot.hAimRPos();
                double tempPosL = robot.hAimLPos();
                if (tempPosR < (1)) {
                    robot.setTurretPos(tempPosR + 0.1, tempPosR + 0.1);
                }
            }

            if (gamepad2.left_bumper) {
                robot.setBackPower(powerTNear);
            }
            if (gamepad2.right_bumper) {
                //robot.setTurretPos(31);
                robot.setAimPos(0.9);
                robot.setBackPower(powerTFar);
            }
            if (gamepad2.dpad_right) {
                robot.setTurretPos(0.5, 0.5);
            }
            if (gamepad2.dpad_right) {
                robot.setIntake1(-.3);

            }

            sleep(50);
        }
    }
}


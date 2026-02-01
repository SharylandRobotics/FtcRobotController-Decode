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

import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team13581.RobotHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@TeleOp(name="Field Centric", group="TeleOp")

public class FieldCentric extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    public static double powerTFar = 0.58;
    public static double powerTNear = 0.5;
    public static double gain = 0.5;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        double axial    = 0;
        double lateral  = 0;
        double yaw      = 0;

        boolean runIntake = false;
        boolean runIntake2 = false;
        boolean runIntakeBackwards = false;

        double lastTargetTurretDeg = 0;
        double lastTargetHeading = 0;

        double velocity = 0;



        robot.init();

        waitForStart();

        while (opModeIsActive()) {
            robot.updateAprilTagDetections();

            axial = -gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;

            robot.teleOpFieldCentric(axial, lateral, yaw);

            telemetry.addData("Controls", "Drive/Strafe: Left Stick | Turn: Right Stick");
            telemetry.addData("Inputs", "axial=%.2f   lateral=%.2f   yaw=%.2f", axial, lateral, yaw);
            telemetry.addData("Error Degree: ", robot.getGoalBearingDeg());
            telemetry.addData("Turret Degree: ", robot.getTurretDegree());
            telemetry.addData("Servo Angle: ", robot.getAimPos());
            telemetry.addData("Turret Pos: ", robot.hAimRPos());
            telemetry.addData("Power: ", robot.getBackPower());
            telemetry.addData("Vel: ", robot.getOuttakeRVel());
            telemetry.addData("Gain: ", gain);
            //telemetry.addData("HSV: ", "H=%.3f S=%.3f V=%.3f", robot.getHSVColor()[0], robot.getHSVColor()[1], robot.getHSVColor()[2]);
            //telemetry.addData("RGB: ", "R=%.3f G=%.3f B=%.3f", robot.getRGBColor()[0], robot.getRGBColor()[1], robot.getRGBColor()[2]);

            telemetry.update();
            if (gamepad1.aWasPressed()){
                runIntake = !runIntake;
                if (runIntake){
                    robot.setIntake1(1);
                }
            }if (gamepad1.xWasPressed()){
                runIntake = !runIntake;
                if (runIntake){
                    robot.setIntake1(1);
                    robot.setIntake2(.7);
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

            if (gamepad1.b){
                robot.setIntake2(0.5);
            }


            if (gamepad1.right_bumper) {
                robot.setIntake2(0);
            }

            if (gamepad2.x) {
                velocity = -700;
            }
            if (gamepad2.a) {
                velocity = 2600;
            }
            if (gamepad2.b) {
                velocity = 0;
            }

            robot.setShootSpeed(velocity);


            if (gamepad2.dpad_up) {
                double tempPos = robot.getAimPos();
                if (tempPos > 0.1) {
                    robot.setAimPos(tempPos - 0.02);
                }
            }

            if (gamepad2.dpad_left) {
                robot.setAimPos(0.45);
            }

            if (gamepad2.dpad_down) {
                double tempPos = robot.getAimPos();
                if (tempPos < (1)) {
                    robot.setAimPos(tempPos + 0.02);
                }
            }


            if (gamepad2.right_trigger > 0) {
                double tempPosR = robot.hAimRPos();

                if (tempPosR > (0)) {
                    robot.setTurretPos(tempPosR - 0.01);
                }
            }
            if (gamepad2.left_trigger > 0)    {
                double tempPosR = robot.hAimRPos();

                if (tempPosR < (1)) {
                    robot.setTurretPos(tempPosR + 0.01);
                }
            }

            if (gamepad2.left_bumper) {
                robot.setAimPos(1);
            }
            if (gamepad2.right_bumper) {
                robot.setAimPos(0);
            }
            if (gamepad2.dpad_right) {
                robot.setTurretPos(0.5);
            }

            if (gamepad2.y){
                double scan = robot.getGoalBearingDeg();
                double deg = 0.0;
                double pos = 0.0;
                if (!Double.isNaN(scan)) {
                    deg = scan + robot.getTurretDegree();
                    pos = robot.hAimRPos() + (scan / 600.0);
                    robot.setTurretPos(pos);
                    //robot.setTurretDegree(scan);
                    telemetry.addData("Correcting by :", scan);
                    telemetry.addData("Absolute Target: ", deg);
                    telemetry.addData("Going to Pos: ", pos);
                    robot.updateAprilTagDetections();
                }
            }
            if (gamepad2.right_stick_button) {
                velocity = 1600;
            }
            if (gamepad2.left_stick_button) {
                velocity = 1300;
            }
            if (gamepad1.dpadUpWasPressed()){
                gain++;
            }
            if (gamepad1.dpadDownWasPressed()){
                gain--;
            }

            //robot.colorSensor.setGain((float) gain);

            sleep(50);
        }
    }
}


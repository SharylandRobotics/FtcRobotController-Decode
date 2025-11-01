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

package org.firstinspires.ftc.team12395.tests; // TODO(STUDENTS): Change to your team package (e.g., org.firstinspires.ftc.team12345.teleop)

import android.graphics.Color;
import android.telephony.IccOpenLogicalChannelResponse;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.team12395.RobotHardware;

import static org.firstinspires.ftc.team12395.RobotHardware.*;

@TeleOp(name="Color Test", group="TeleOp")
@Config
@Disabled
// TODO(STUDENTS): You may rename this for your robot (e.g., "Field Centric - Comp Bot)
public class ColorTest extends LinearOpMode {

    // NOTE: One hardware instance per OpMode keeps mapping/IMU use simple and testable
    RobotHardware robot = new RobotHardware(this);

    float[] hsvValues = new float[3];

    @Override
    public void runOpMode() {
        char scannedColor;
        int[] solution = null;
        int spinAngle;
        String chamberString = "_  ";

        // Driver inputs (range roughly [-1, 1])
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init();

        waitForStart();

        // --- TELEOP LOOP ---
        while (opModeIsActive()) {
            Color.RGBToHSV(robot.colorSensor.red(), robot.colorSensor.green(), robot.colorSensor.blue(), hsvValues);
            if (gamepad2.a){
                spinAngle = 60;
            } else {
                spinAngle = 120;
            }
            if (!robot.spindexer.isBusy()) {
                if (gamepad2.leftBumperWasPressed()) { // ccw
                    robot.spindexerHandler( spinAngle);
                } else if (gamepad2.rightBumperWasPressed()) { // cw
                    robot.spindexerHandler( -spinAngle);
                }

                chamberString = "UUU";
                StringBuilder chamberBuilder = new StringBuilder(chamberString);
                chamberBuilder.setCharAt(robot.chamber, 'V');
                chamberString = chamberBuilder.toString();

                if (gamepad1.xWasPressed()){
                    scannedColor = robot.scanColor();

                    if (scannedColor != '0'){
                        StringBuilder magBuilder = new StringBuilder(mag);
                        magBuilder.setCharAt(robot.chamber, scannedColor);
                        mag = magBuilder.toString();

                        //

                    }
                }

                if (gamepad1.yWasPressed()){
                    solution = robot.solvePattern();
                }
            }


            telemetry.addData("current Chamber: ", robot.chamber);
            telemetry.addData("chamber pic: ", chamberString);
            telemetry.addData("current Mag: ", mag);
            telemetry.addData("current Pattern: ", pattern);
            if (solution != null) {
                telemetry.addData("current solution: ", solution[0] + "120 deg, then " + solution[1] + " 120 deg");
            } else {
                telemetry.addData("current solution: ", "no solution");
            }
            telemetry.addData("Colors (HSV): ",  "H=%.3f S=%.3f V=%.3f ", hsvValues[0], hsvValues[1], hsvValues[2]);
            telemetry.addData("Colors (HSV): ",  "R=%d%n G=%d%n B=%d%n ", robot.colorSensor.red(), robot.colorSensor.green(), robot.colorSensor.blue());
            telemetry.update();

            // Pace loop-helps with readability and prevents spamming the DS
            sleep(50); // ~20 Hz;
        }
    }
}


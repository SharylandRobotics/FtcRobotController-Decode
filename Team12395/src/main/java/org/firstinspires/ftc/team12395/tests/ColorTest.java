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
// TODO(STUDENTS): You may rename this for your robot (e.g., "Field Centric - Comp Bot)
public class ColorTest extends LinearOpMode {

    // NOTE: One hardware instance per OpMode keeps mapping/IMU use simple and testable
    RobotHardware robot = new RobotHardware(this);

    float[] hsvValues = new float[3];

    public static float gain = 100;
    public static boolean testColor = false;
    public static boolean shoot = false;

    @Override
    public void runOpMode() {

        double cT = 0;
        double prT = 0;

        boolean runClock = false;
        int clock = 0;

        // Driver inputs (range roughly [-1, 1])
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init();

        mag = "000";

        waitForStart();


        // --- TELEOP LOOP ---
        while (opModeIsActive()) {
            NormalizedRGBA color = robot.colorSensor.getNormalizedColors();
            Color.colorToHSV(color.toColor(), hsvValues);
            if (testColor){
                robot.setIntakeSpeed(-800);
            }

            String colorS = "UNKNOWN/ROTATING";
            if (shoot && !mag.contains("0") && !robot.spindexer.isBusy()){
                robot.spindexerHandler(-480);
                robot.setMagManualBulk("000");
            } else if (testColor && !robot.spindexer.isBusy() && !runClock && mag.contains("0")){
                if (hsvValues[2] < 0.15){
                    colorS = "NONE";
                } else if (hsvValues[0] > 200 && hsvValues[0] <= 240){
                    colorS = "PURPLE";
                    if (mag.charAt(chamber) == '0') {
                        robot.setChamberManual('P');
                    }
                    runClock = true;
                } else if (hsvValues[1] > 0.6 && hsvValues[1] < 0.75){
                    colorS = "GREEN";
                    if (mag.charAt(chamber) == '0') {
                        robot.setChamberManual('G');
                    }
                    runClock = true;
                }
            }

            if (runClock && clock < 5){
                clock++;
            }

            if (clock == 4){
                robot.spindexerHandler(120);
                clock = 0;
                runClock = false;
            }

            telemetry.addData("Color is: ", colorS);

            robot.colorSensor.setGain(gain);

            cT = getRuntime();
            telemetry.addData("Delay from previous update (s): ", cT - prT);
            prT = cT;
            telemetry.addData("Colors (HSV): ",  "H=%.3f S=%.3f V=%.3f ", hsvValues[0], hsvValues[1], hsvValues[2]);
            telemetry.addData("float Colors (RGB): ",  "R=%.3f G=%.3f B=%.3f ", color.red, color.green, color.blue);
            telemetry.addData(robot.getMagPicture(),"");
            telemetry.update();

            // Pace loop-helps with readability and prevents spamming the DS
            sleep(50); // ~20 Hz;
        }
    }
}


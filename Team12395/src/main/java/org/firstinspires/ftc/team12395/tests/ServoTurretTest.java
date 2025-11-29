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

@TeleOp(name="Turret Test", group="TeleOp")
@Config
@Disabled
// TODO(STUDENTS): You may rename this for your robot (e.g., "Field Centric - Comp Bot)
public class ServoTurretTest extends LinearOpMode {

    // NOTE: One hardware instance per OpMode keeps mapping/IMU use simple and testable
    RobotHardware robot = new RobotHardware(this);

    public static int target = 0;
    public static boolean run = false;

    @Override
    public void runOpMode() {

        // Driver inputs (range roughly [-1, 1])
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init();

        waitForStart();

        // --- TELEOP LOOP ---
        while (opModeIsActive()) {
            if (run) {
                robot.setTurretHandlerAbsolute(target);
                if (robot.turretHandler.runToTarget()){
                    run = false;
                    telemetry.addData("DONE", "");
                }
            } else {
                if (gamepad1.x){
                    robot.turretR.setPower(0.5);
                    robot.turretL.setPower(0.5);
                } else if (gamepad1.b) {
                    robot.turretL.setPower(-0.5);
                    robot.turretR.setPower(-0.5);
                } else {
                    robot.turretL.setPower(0);
                    robot.turretR.setPower(0);
                }


            }
            if (gamepad1.a){
                run = false;
            } else if (gamepad1.dpad_up){
                run = true;
            }
            robot.turretHandler.setPGain();
            robot.turretHandler.setIGain();
            robot.turretHandler.setDGain();

            telemetry.update();

            // Pace loop-helps with readability and prevents spamming the DS
            telemetry.addData("Pos: ", robot.turretHandler.getCurrentPosition());
            telemetry.addData("Power: ", robot.turretHandler.getServoPower());
            telemetry.addData("Error Deg: ", robot.turretHandler.getCurrentError()/robot.turretTicksPerDegree);
            telemetry.addData("Output: ", robot.turretHandler.getOutput());
            sleep(50); // ~20 Hz;
        }
    }
}


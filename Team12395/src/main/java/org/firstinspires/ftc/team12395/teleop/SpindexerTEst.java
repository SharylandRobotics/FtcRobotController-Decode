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

package org.firstinspires.ftc.team12395.teleop; // TODO(STUDENTS): Change to your team package (e.g., org.firstinspires.ftc.team12345.teleop)

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.team12395.RobotHardware;

@TeleOp(name="Spindexer Test", group="TeleOp")
@Config
// TODO(STUDENTS): You may rename this for your robot (e.g., "Field Centric - Comp Bot)
public class SpindexerTEst extends LinearOpMode {

    // NOTE: One hardware instance per OpMode keeps mapping/IMU use simple and testable
    RobotHardware robot = new RobotHardware(this);

    public static int targetPos = 360;
    public static double currentPos;
    public static double P;
    //public static double I;
    //public static double D;
    //public static double F;


    @Override
    public void runOpMode() {
        // Driver inputs (range roughly [-1, 1])
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init();

        P = robot.spindexer.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).p;
        //I = robot.spindexer.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).i;
        //D = robot.spindexer.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).d;
        //F = robot.spindexer.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f;

        waitForStart();

        robot.spindexer.setTargetPosition(targetPos);

        // --- TELEOP LOOP ---
        while (opModeIsActive()) {

            if (!robot.spindexer.isBusy()) {
                robot.spindexer.setTargetPosition(targetPos);
                robot.spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (targetPos == 360){
                    targetPos = 0;
                } else if (targetPos == 0){
                    targetPos = 360;
                }
            }
            currentPos = robot.spindexer.getCurrentPosition();

            robot.spindexer.setPositionPIDFCoefficients(P);

            telemetry.addData("target Position: ", targetPos);
            telemetry.addData("current Position: ", currentPos);
            telemetry.addData("PIDF: ", robot.spindexer.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
            telemetry.update();

            // Pace loop-helps with readability and prevents spamming the DS
            sleep(50); // ~20 Hz;
        }
    }
}


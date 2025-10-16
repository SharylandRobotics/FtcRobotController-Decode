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

package org.firstinspires.ftc.team13588.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team13588.RobotHardware;

@TeleOp(name = "Robot Centric", group = "opMode")

public class RobotCentric extends LinearOpMode {

    // Create hardware instance and pass current OpMode reference
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {

        // Variables for joystick input: forward/back (axial), strafe (lateral), rotation (yaw)
        double axial;
        double lateral;
        double yaw;

        // Initialize all motors, IMU, and hardware configuration
        robot.init();

        while(opModeInInit()) {
            // Display IMU heading and init status on Driver Station until start
            telemetry.addData("Status", "Hardware Initialized");
            telemetry.addData("Heading", "%4.0f", robot.getHeading());
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // Main control loop: continuously while TeleOp is active
        while (opModeIsActive()) {

            boolean slow = gamepad1.left_bumper;
            double scale = slow ? 0.4 : 1.0;

            // Read real-time joystick values from gamepad
            axial = -gamepad1.left_stick_y * scale;
            lateral = gamepad1.left_stick_x * scale;
            yaw = gamepad1.right_stick_x * scale;

            // Apply joystick inputs directly to robot-centric drive control
            robot.driveRobotCentric(axial, lateral, yaw);

            // Display control instructions and current input values to Driver Station
            telemetry.addData("Controls", "Drive/Strafe: Left Stick | Turn: Right Stick");
            telemetry.addData("Mode", slow ? "SLOW" : "NORMAL");
            telemetry.addData("Inputs", "axial=%.2f   lateral=%.2f   yaw=%.2f", axial, lateral, yaw);
            telemetry.update();

            // Small delay to prevent telemetry flooding
            sleep(50);
        }
    }
}
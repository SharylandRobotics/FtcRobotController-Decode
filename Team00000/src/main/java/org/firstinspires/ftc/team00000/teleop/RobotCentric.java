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
 * CAUSED ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Student notes:
// • Robot-centric TeleOp: left stick = drive/strafe (robot frame), right stick = turn
// • Uses RobotHardware for motors and IMU
// • Hold left bumper for slow mode
// • Forward and turn directions are relative to the robot (not the field)
// • TODO (student): Adjust slow-mode speed (scale) and add joystick deadbands if needed

package org.firstinspires.ftc.team00000.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team00000.RobotHardware;

@TeleOp(name = "Robot Centric", group = "opMode")

public class RobotCentric extends LinearOpMode {

    // Hardware reference for drive and sensors
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        // Driver inputs: axial (forward/back), lateral (strafe), yaw (turn)
        double axial;
        double lateral;
        double yaw;

        // Set up motors, encoders, and IMU before starting TeleOp
        robot.init();

        while(opModeInInit()) {
            // Show IMU heading and confirm initialization before pressing START
            telemetry.addData("Status", "Hardware Initialized");
            telemetry.addData("Heading", "%4.0f", robot.getHeading());
            telemetry.update();
        }

        // Waits for the driver to press START
        waitForStart();
        if (isStopRequested()) return;

        // Main control loop: continuously while TeleOp is active
        while (opModeIsActive()) {
            // Hold left bumper for slow mode (precision control)
            boolean slow = gamepad1.left_bumper;
            // TODO (student): Change 0.4 to your preferred slow-mode speed
            double scale = slow ? 0.4 : 1.0;

            // TODO (student): Add joystick deadbands (e.g., if |value| < 0.05 → 0) to filter small stick noise
            axial = -gamepad1.left_stick_y * scale;
            lateral = gamepad1.left_stick_x * scale;
            yaw = gamepad1.right_stick_x * scale;

            // TODO (student): Reverse any axis here if movement feels opposite to the stick
            robot.driveRobotCentric(axial, lateral, yaw);

            // TODO (student): Add encoder or heading data here for debugging (e.g., motor positions, IMU yaw)
            // Display controls and live joystick data on Driver Station
            telemetry.addData("Controls", "Drive/Strafe: Left Stick | Turn: Right Stick");
            telemetry.addData("Mode", slow ? "SLOW" : "NORMAL");
            telemetry.addData("Inputs", "axial=%.2f   lateral=%.2f   yaw=%.2f", axial, lateral, yaw);
            telemetry.update();

            // Small pause to keep telemetry updates smooth
            sleep(50);
        }
    }
}

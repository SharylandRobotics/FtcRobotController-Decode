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

// Student notes:
// • Field-centric TeleOp: left stick = drive/strafe (field-aligned), right stick = turn
// • Right bumper = AprilTag auto-assist (drives to a chosen tag)
// • This OpMode uses RobotHardware for motors, IMU, and vision
// • If you change wheel size or gains, update them in RobotHardware
// • TODO (student): Update slow‑mode scale for your field/driver

package org.firstinspires.ftc.team00000.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.team00000.RobotHardware;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "Field Centric", group = "opMode")

public class FieldCentric extends LinearOpMode {

    // Hardware reference (motors, IMU, camera controls)
    // Create hardware instance and pass current OpMode reference
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {

        // Driver inputs: axial (forward/back), lateral (strafe), yaw (turn)
        boolean targetFound;
        double axial;
        double lateral;
        double yaw;

        // Set up motors/IMU, then build AprilTag camera pipeline
        // Initialize all motors, IMU, and hardware configuration
        robot.init();
        // TODO (student): If not using AprilTags, you can remove initAprilTag() and the auto‑assist block
        robot.initAprilTag();

        while(opModeInInit()) {
            // Show heading and camera hint until START is pressed
            // Display IMU heading and init status on Driver Station until start
            telemetry.addData("Status", "Hardware Initialized");
            telemetry.addData("Heading", "%4.0f", robot.getHeading());
            telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
            telemetry.addData(">", "Touch START to start OpMode");
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // Main control loop: continuously while TeleOp is active
        while (opModeIsActive()) {

            targetFound = false;
            robot.desiredTag = null;

            List<AprilTagDetection> currentDetections = robot.aprilTag.getDetections();
            // Look for a valid target tag from the current camera detections
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if((detection.id == 20 || detection.id == 24)) {
                        targetFound = true;
                        robot.desiredTag = detection;
                        break;
                    }
                }
            }

            // Hold left bumper for slow mode (precision driving)
            boolean slow = gamepad1.left_bumper;
            // TODO (student): Change 0.4 to your preferred slow‑mode speed
            double scale = slow ? 0.4 : 1.0;

            // Hold right bumper to auto-drive toward the selected AprilTag
            if (gamepad1.right_bumper && targetFound) {

                // TODO (student): Verify your camera orientation; if signs feel wrong, flip lateral/yaw
                // Camera-based errors (units from ftcPose):
                //  rangeError (in): forward/back distance to target vs DESIRED_DISTANCE
                //  headingError (deg): how much to turn to face the tag
                //  yawError (deg): sideways offset relative to the tag
                double rangeError = (robot.desiredTag.ftcPose.range - robot.DESIRED_DISTANCE);
                double headingError = robot.desiredTag.ftcPose.bearing;
                double yawError = robot.desiredTag.ftcPose.yaw;

                // TODO (student): Tune GAINS in RobotHardware to reduce wobble and overshoot
                // TODO (student): Add an arrival check: if |rangeError|<1, |headingError|<2, |yawError|<2 → set all to 0
                // Map errors → drive axes (signs chosen to feel natural)
                axial = Range.clip(rangeError * RobotHardware.AXIAL_GAIN, -RobotHardware.AXIAL_SPEED, RobotHardware.AXIAL_SPEED);
                lateral = Range.clip(yawError * RobotHardware.LATERAL_GAIN, -RobotHardware.LATERAL_SPEED, RobotHardware.LATERAL_SPEED);
                yaw = Range.clip(-headingError * RobotHardware.YAW_GAIN, -RobotHardware.YAW_SPEED, RobotHardware.YAW_SPEED);

                robot.driveRobotCentric(axial, lateral, yaw);
            } else {

                // TODO (student): Add joystick deadbands (e.g., if |value| < 0.05 → 0) to ignore small noise
                // Otherwise, use driver sticks (field-centric)
                // Field-centric inputs: up = field forward. Left stick Y is inverted.
                // Read real-time joystick values from gamepad
                axial = -gamepad1.left_stick_y * scale;
                lateral = gamepad1.left_stick_x * scale;
                yaw = gamepad1.right_stick_x * scale;

                robot.driveFieldCentric(axial, lateral, yaw);
            }

            // TODO (student): Add tag ID/range/bearing to telemetry to help tuning
            // Driver Station info (controls + current input)
            // Display control instructions and current input values to Driver Station
            telemetry.addData("Controls", "Drive/Strafe: Left Stick | Turn: Right Stick");
            telemetry.addData("Mode", slow ? "SLOW" : "NORMAL");
            if (gamepad1.right_bumper && targetFound) {
                telemetry.addData("Auto", "axial=%.2f   lateral=%.2f   yaw=%.2f", axial, lateral, yaw);
            } else {
                telemetry.addData("Manual", "axial=%.2f   lateral=%.2f   yaw=%.2f", axial, lateral, yaw);
            }
            telemetry.addData("\n>", "HOLD Right-Bumper to Drive to Target\n");
            telemetry.update();

            // Small pause to keep telemetry responsive
            sleep(50);
        }
    }
}

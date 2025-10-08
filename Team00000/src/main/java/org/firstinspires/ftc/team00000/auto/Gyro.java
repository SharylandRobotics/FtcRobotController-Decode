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
// • Example Autonomous using gyro (IMU) to drive and turn accurately
// • Demonstrates use of driveStraight(), turnToHeading(), and holdHeading()
// • All motion helpers come from RobotHardware.java
// • Make sure the robot is on a flat surface before running
// • TODO (student): Adjust distances/angles below and tune speeds/gains in RobotHardware for YOUR robot

package org.firstinspires.ftc.team00000.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team00000.RobotHardware;

import static org.firstinspires.ftc.team00000.RobotHardware.*;

@Autonomous(name = "Gyro", group = "opMode")

// Autonomous routine using gyro-based driving with RobotHardware helpers
public class Gyro extends LinearOpMode {

    // Link RobotHardware to this OpMode
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {

        // Initialize motors and IMU before starting Autonomous
        robot.init();

        while(opModeInInit()) {
            // Show heading and confirm initialization until START is pressed
            telemetry.addData("Status", "Hardware Initialized");
            telemetry.addData("Heading", "%4.0f", robot.getHeading());
            telemetry.update();
        }

        // Wait for driver to press START
        waitForStart();
        if (isStopRequested()) return;

        // Run once after START (main autonomous sequence)
        if (opModeIsActive()) {

            // Example path:
            // 1. Drive forward 24"
            // 2. Turn -45°, hold
            // 3. Drive 17" forward at -45°
            // 4. Turn 45°, hold
            // 5. Drive 17" forward at 45°
            // 6. Realign to 0° and return backward 48"

            // TODO (student): Set forward distance (inches). Reduce AXIAL_SPEED if wheels slip.
            // TODO (student): If your start heading isn't field 0°, re-zero yaw at the start of this OpMode.
            robot.driveStraight(AXIAL_SPEED, 24.0, 0.0);

            // TODO (student): Set turn angle (deg). Increase YAW_SPEED carefully to avoid overshoot.
            robot.turnToHeading(YAW_SPEED, -45.0);
            // TODO (student): Hold time (sec) lets the robot settle before the next move.
            robot.holdHeading(YAW_SPEED, -45.0, 0.5);

            // TODO (student): Drive while holding current heading (here: -45°). Adjust distance as needed.
            robot.driveStraight(AXIAL_SPEED, 17.0, -45.0);
            // TODO (student): Second turn. Positive angles turn CCW (left) if your IMU uses CCW-positive.
            robot.turnToHeading(YAW_SPEED, 45.0);
            // TODO (student): Short settle to improve next straight accuracy.
            robot.holdHeading(YAW_SPEED, 45.0, 0.5);

            // TODO (student): Another straight leg at +45°. Keep speed under control on slick tiles.
            robot.driveStraight(AXIAL_SPEED, 17.0, 45.0);
            // TODO (student): Realign to 0° so the last leg returns along field-forward.
            robot.turnToHeading(YAW_SPEED, 0.0);
            // TODO (student): Longer settle before the return leg (useful if battery sag causes drift).
            robot.holdHeading(YAW_SPEED, 0.0, 1.0);

            // TODO (student): Negative distance drives backward. Verify clearance behind the robot.
            robot.driveStraight(AXIAL_SPEED, -48.0, 0.0);

            // TODO (student): Optional customizations:
            //  • Add sleep(ms) between steps if your robot oscillates after moves
            //  • Add early-exit checks: if (isStopRequested()) return; between each step
            //  • Add park or scoring actions here (intake/arm/servo commands)
            //  • Convert inches to cm if your team prefers metric
            //  • Replace this sequence with Road Runner trajectories later

            // Show that the path is complete on Driver Station
            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);  // Pause to display last telemetry message.
        }
    }
}

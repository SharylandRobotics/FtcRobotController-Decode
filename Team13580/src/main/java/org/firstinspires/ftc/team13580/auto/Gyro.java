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

package org.firstinspires.ftc.team13580.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team13580.RobotHardware;

import static org.firstinspires.ftc.team13580.RobotHardware.*;

@Autonomous(name = "Gyro", group = "opMode")

// Autonomous routine using gyro-based driving with RobotHardware helpers
public class Gyro extends LinearOpMode {

    // Instantiate RobotHardware and link this OpMode
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {

        // Initialize all motors and IMU before start
        robot.init();

        while(opModeInInit()) {
            // Display heading and status continuously during init loop
            telemetry.addData("Status", "Hardware Initialized");
            telemetry.addData("Heading", "%4.0f", robot.getHeading());
            telemetry.update();
        }

        // Wait for PLAY; exit early if stop is pressed
        waitForStart();
        if (isStopRequested()) return;

        // Execute full autonomous path sequence once started
        if (opModeIsActive()) {
            robot.setOuttakePower(0.7);

            // Drive 24" forward, then turn and hold headings as defined
            robot.driveStraight(MAX_AUTO_AXIAL, -24.0, 0.0);
            robot.setOuttakePower(1.0);
            //robot.turnToHeading(MAX_AUTO_YAW, -45.0);
           // robot.holdHeading(MAX_AUTO_YAW, -45.0, 0.5);
            sleep(2000);
            robot.setOuttakePower(1.0);
            robot.setKickerLeftPosition(.35);
            robot.setKickerPosition(0);
            sleep(3000);
            robot.setOuttakePower(0);
            robot.driveStraight(MAX_AUTO_AXIAL, -24.0, 0.0);
          //  robot.turnToHeading(MAX_AUTO_YAW,45);
           // robot.holdHeading(MAX_AuTO_YAW,);



            //robot.driveStraight(MAX_AUTO_AXIAL, 17.0, -45.0);
            //robot.turnToHeading(MAX_AUTO_YAW, 45.0);
            //robot.holdHeading(MAX_AUTO_YAW, 45.0, 0.5);

            //robot.driveStraight(MAX_AUTO_AXIAL, 17.0, 45.0);
            //robot.turnToHeading(MAX_AUTO_YAW, 0.0);f
            //robot.holdHeading(MAX_AUTO_YAW, 0.0, 1.0);

            //robot.driveStraight(MAX_AUTO_AXIAL, -48.0, 0.0);

            // Indicate completion and pause for display
            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);  // Pause to display last telemetry message.
        }
    }
}

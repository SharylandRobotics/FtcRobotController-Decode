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

package org.firstinspires.ftc.team12397.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team12397.RobotHardware;

import static org.firstinspires.ftc.team12397.RobotHardware.AXIAL_SPEED;

@Autonomous(name = "TopBlue", group = "opMode")

// Autonomous routine using gyro-based driving with RobotHardware helpers
public class
TopBlue extends LinearOpMode {


    // Instantiate RobotHardware and link this OpMode
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {

        // Initialize all motors and IMU before start
        robot.init();

        while (opModeInInit()) {
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

            Boolean aprilTagFound = false;

            Boolean phase1 = true;
            Boolean phase2 = false;
            Boolean phase3 = false;

            while(phase1){
                robot.turretPower(.70);
                robot.driveStraight(1, -50, 0);
                robot.holdHeading(robot.YAW_SPEED, 0, 2);
                robot.turnToHeading(1,-5);
                robot.holdHeading(robot.YAW_SPEED, -5, 1);

                robot.setIntakeServo(0);
                robot.holdHeading(robot.YAW_SPEED, -5, 2);
                robot.setIntakeServo(1);
                robot.intakePower(-.3);
                robot.holdHeading(robot.YAW_SPEED, -5, 2);
                robot.setIntakeServo(0);
                robot.holdHeading(robot.YAW_SPEED, -5, 2);

                robot.setIntakeServo(1);

                // turn off motors
                robot.turretPower(0);
                robot.intakePower(0);
                //phase 2

                robot.turnToHeading(1,40);
                robot.holdHeading(robot.YAW_SPEED, 40, 1);
                robot.straif(1,-6,40);
                robot.intakePower(-.5);
                robot.driveStraight(1, 20, 40);
                robot.driveStraight(.5,13,40);

                robot.holdHeading(robot.YAW_SPEED, 40, 2);

                robot.intakePower(0);
                robot.turretPower(.80);
                robot.driveStraight(1,-31,40);
                robot.holdHeading(robot.YAW_SPEED, 40, 2);

                robot.turnToHeading(1,-5);
                robot.holdHeading(robot.YAW_SPEED, -5, 2);

                robot.setIntakeServo(0);
                robot.holdHeading(robot.YAW_SPEED, -5, 2);

                robot.setIntakeServo(1);
                robot.intakePower(-.3);
                robot.holdHeading(robot.YAW_SPEED, -5, 2);

                robot.setIntakeServo(0);
                robot.holdHeading(robot.YAW_SPEED, -5, 2);


                // turn off motors
                robot.setIntakeServo(1);
                robot.turretPower(0);
                robot.intakePower(0);

                phase1 = false;





            }



        }
    }
    public void shootMotorsAuto(){
        robot.driveStraight(AXIAL_SPEED, -55, 0);
        robot.straif(1,5,0);
        robot.turnToHeading(AXIAL_SPEED, 30);
        //shoot motors
        robot.turretPower(0);
        robot.intakePower(0);

        robot.turretPower(.85);
        robot.holdHeading(robot.YAW_SPEED, 0, 3);

        robot.setIntakeServo(0);
        robot.holdHeading(robot.YAW_SPEED, 0, 3);
        robot.setIntakeServo(1);
        robot.intakePower(-.3);
        robot.holdHeading(robot.YAW_SPEED, 0, 2);
        robot.setIntakeServo(0);
        robot.holdHeading(robot.YAW_SPEED, 0, 3);

        robot.setIntakeServo(1);


        // turn off motors
        robot.turretPower(0);
        robot.intakePower(0);
        //move
        robot.straif(1,-25,0);
    }
}




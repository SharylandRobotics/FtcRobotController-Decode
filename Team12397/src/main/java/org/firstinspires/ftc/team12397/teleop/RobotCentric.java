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

package org.firstinspires.ftc.team12397.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team12397.RobotHardware;

@TeleOp(name="Robot Centric", group="TeleOp")
// TODO(STUDENTS): You may rename this OpMode in the annotation for clarity (e.g., "Robot-Centric - Practice Bot")
public class RobotCentric extends LinearOpMode {

    // NOTE: One RobotHardware instance per OpMode keeps mapping/telemetry simple.
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        // Driver inputs (range about [-1, 1])
        double axial    = 0; // forward/back (+ forward)
        double lateral  = 0; // strafe left/right (+ right)
        double yaw      = 0; // rotation (+ CCW/left turn)
        // --- INIT PHASE ---
        // WHY: Centralized initialization (motor directions, encoders, IMU) lives in RobotHardware.init()
        // TODO(STUDENTS): Confirm motor names, directions, and zero-power modes in RobotHardware.init()
        robot.init();

        // Wait for START on the Diver Station
        waitForStart();

        // --- TELEOP LOOP ---
        while (opModeIsActive()) {

            // Read sticks (FTC gamepads: pushing left_stick_y forward is negative → invert it)
            axial   = -gamepad1.left_stick_y;
            lateral =  gamepad1.left_stick_x;
            yaw     =  gamepad1.right_stick_x;


            //servo Hood stuff starts here
            // old code
            /*if (gamepad1.right_bumper)
                robot.setHoodPositions(.5);

            if (gamepad1.left_bumper)
                robot.setHoodPositions(0.0);

             */
            //servo hood
            boolean servoOn = false;
            boolean lastServoState = false;
            boolean currentServoState = (gamepad1.bWasPressed());
            if(currentServoState && !lastServoState){
                servoOn = !servoOn;
            }
            if(servoOn){
                robot.setHoodPositions(.5);
            }
            else{
                robot.setHoodPositions(0.0);
            }
            lastServoState = currentServoState;

            // Pace this loop so hands move at a reasonable speed.

//            boolean intakeservoOn = false;
//            boolean intakelastServoState = false;
//            boolean intakecurrentServoState = (gamepad1.y);
//            if(intakecurrentServoState && !intakelastServoState){
//                intakeservoOn = !intakeservoOn;
//            }
//            if(intakeservoOn){
//                robot.setIntakeServo(.5);
//            }
//            else{
//                robot.setIntakeServo(0.0);
//            }
//            intakelastServoState = intakecurrentServoState;

            // Pace this loop so hands move at a reasonable speed.

            // turret motor
            boolean turretOn = false;
            boolean lastTriggerState = false;
            boolean currentTriggerState = (gamepad1.rightBumperWasReleased());
            if(currentTriggerState && !lastTriggerState){
                turretOn = !turretOn;
            }
            if(turretOn){
                robot.turretPower(1);
            }
            else{
                robot.turretPower(0);
            }
            lastTriggerState = currentTriggerState;
            //intake motor
            boolean intakeMotorOn = false;
            boolean intakeMotorlastTriggerState = false;
            boolean intakeMotorcurrentTriggerState = (gamepad1.left_trigger > .5);
            if(intakeMotorcurrentTriggerState && !intakeMotorlastTriggerState){
                intakeMotorOn = !intakeMotorOn;
            }
//            if(intakeMotorOn){
//                robot.intakePower(-1);
//            }
//            else{
//                robot.intakePower(0);
//            }
            intakeMotorlastTriggerState = intakeMotorcurrentTriggerState;
           /* if (gamepad1.right_trigger > 0){
                robot.turretPower(1);
            }
              if(gamepad1.left_trigger > 0){
                robot.turretPower(0);
            }
             */
            // Optional: Deadband to filter tiny stick noise (uncomment to use)
            // double dead = 0.05; // TODO(STUDENTS): tune
            // axial   = (Math.abs(axial)   < dead) ? 0 : axial;
            // lateral = (Math.abs(lateral) < dead) ? 0 : lateral;
            // yaw     = (Math.abs(yaw)     < dead) ? 0 : yaw;

            // Optional: Precision/slow mode (hold Left Trigger to reduce overall sensitivity)
            // double slow = 1.0 - (0.6 * gamepad1.left_trigger); // 1.0 → 0.4 as LT goes 0→1
            // axial *= slow; lateral *= slow; yaw *= slow;

            // WHY: Robot-centric uses the driver’s frame (no IMU rotation); great for quick testing.
            robot.driveRobotCentric(axial, lateral, yaw);


            // Telemetry for drivers + debugging
            telemetry.addData("Controls", "Drive/Strafe: Left Stick | Turn: Right Stick");
            telemetry.addData("Inputs", "axial=%.2f   lateral=%.2f   yaw=%.2f", axial, lateral, yaw);
            // Optional: expose heading during tuning
            // telemetry.addData("Heading(rad)", robot.getHeadingRadians()); / add a getter in RobotHardware if desired
            //servo stuff
            telemetry.addData("Drive", "Left Stick");
            telemetry.addData("Turn", "Right Stick");
            telemetry.addData("Hood Servo Up/Down", "Button B toggle");
            telemetry.addData("intake Servo Up/Down", "Button Y toggle");


            telemetry.addData("-", "-------");
            telemetry.addData("Turret motor shooter", "Right Bumper toggle");
            telemetry.addData("intake motor", "Right Trigger Hold");


            telemetry.update();

            // Pace loop-helps with readability and prevents spamming the DS
            sleep(50); // ~20 Hz; TODO(STUDENTS): adjust for your robot feel (e.g., 20-30 ms for snappier control)
        }
    }
}


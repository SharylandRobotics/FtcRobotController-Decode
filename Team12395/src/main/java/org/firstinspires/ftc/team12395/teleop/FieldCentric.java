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

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.team12395.RobotHardware;

@TeleOp(name="Field Centric", group="TeleOp")
@Config
// TODO(STUDENTS): You may rename this for your robot (e.g., "Field Centric - Comp Bot)
public class FieldCentric extends LinearOpMode {

    // NOTE: One hardware instance per OpMode keeps mapping/IMU use simple and testable
    RobotHardware robot = new RobotHardware(this);

    public static double velocity = 0;
    public static double angle = 0.1;

    public static double slewTarget = 0;
    public static double maxTurn = 90;

    public static double indexerTarget = 0;

    public static double counterFlipper = 0;

    public static double preSetVelocity = 1100;
    public static double preSetAngleFar = 0.2;
    public static double preSetAngleClose = 0.5;





    @Override
    public void runOpMode() {
        // Driver inputs (range roughly [-1, 1])
        double axial    = 0; // forward/back (+ forward)
        double lateral  = 0; // strafe left/right (+ right)
        double yaw      = 0; // rotation (+ CCW/left turn)

        double slewRate = 0;

        robot.init();

        waitForStart();

        // --- TELEOP LOOP ---
        while (opModeIsActive()) {

            axial   = -gamepad1.left_stick_y;
            lateral =  gamepad1.left_stick_x;
            yaw     =  gamepad1.right_stick_x;


            robot.driveFieldCentric(axial, lateral, yaw);

            slewRate = Math.abs(gamepad1.right_trigger - gamepad1.left_trigger)*0.125;
            slewTarget = (gamepad1.left_trigger > 0 ? -maxTurn : (gamepad1.right_trigger > 0 ? maxTurn : robot.getCurrentTurretDegreePos() ));

            robot.setTurretPositionAbsolute(slewTarget, slewRate);

            velocity += (gamepad1.dpad_up ? 10 : 0) - (gamepad1.dpad_down ? 10 : 0);
            if (gamepad1.xWasPressed()){
                velocity = preSetVelocity;
            } else if (gamepad1.bWasPressed()){
                velocity = 0;
            }

            angle += (gamepad1.dpad_left ? 0.045 : 0) - (gamepad1.dpad_right ? 0.045 : 0);
            angle = Range.clip(angle, 0.1, 1);

            robot.setShooterVelocity(velocity);
            robot.setHoodAngle(angle);

            indexerTarget = (gamepad2.right_stick_x < 0 ? -360.5 : (gamepad2.right_stick_x > 0 ? 360.5 : robot.getSpindexerAzimuth()[1] ));

            if (gamepad2.bWasPressed()){
                counterFlipper = counterFlipper >= 1 ? 0 : 1;
            }
            robot.setCounterRotatePos(counterFlipper);

            robot.setSpindexerAbsoluteCircleAngle(indexerTarget);

            // Telemetry for drivers + debugging
            telemetry.addData("Controls", "Drive/Strafe: Left Stick | Turn: Right Stick");
            telemetry.addData("Inputs", "angle=%.2f   velocity=%.2f", angle, velocity);
            telemetry.addData("Measured Velocity: ", robot.shooter.getVelocity());
            // telemetry.addData("Heading(rad)", robot.getHeadingRadians()); / add a getter in RobotHardware if desired
            telemetry.update();

            // Pace loop-helps with readability and prevents spamming the DS
            sleep(50); // ~20 Hz;
        }
    }
}


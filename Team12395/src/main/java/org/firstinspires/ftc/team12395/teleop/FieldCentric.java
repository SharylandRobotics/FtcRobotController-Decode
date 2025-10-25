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

import static org.firstinspires.ftc.team12395.RobotHardware.*;

@TeleOp(name="Field Centric", group="TeleOp")
@Config
public class FieldCentric extends LinearOpMode {

    // NOTE: One hardware instance per OpMode keeps mapping/IMU use simple and testable
    RobotHardware robot = new RobotHardware(this);

    public static double velocity = 0;
    public static double angle = 0.1;

    public static double slewTarget = 0;

    public static double indexerTarget = 0;

    public static double preSetVelocity = 1100;
    public static double preSetAngleFar = 0.69;
    public static double preSetAngleClose = 0.2;

    public static double armPos = 1;

    public static int intakeVel = 0;





    @Override
    public void runOpMode() {
        // Driver inputs (range roughly [-1, 1])
        double axial    = 0; // forward/back (+ forward)
        double lateral  = 0; // strafe left/right (+ right)
        double yaw      = 0; // rotation (+ CCW/left turn)

        double spinAngle = 120;

        double slewRate = 0;

        boolean checkSpinBusy = false;
        double armClock = 11;

        robot.init();

        robot.limelight.start();
        robot.limelight.pipelineSwitch(2);

        waitForStart();

        robot.colorSensor.enableLed(true);

        // --- TELEOP LOOP ---
        while (opModeIsActive()) {

            axial   = -gamepad1.left_stick_y;
            lateral =  gamepad1.left_stick_x;
            yaw     =  gamepad1.right_stick_x;




            robot.driveFieldCentric(axial, lateral, yaw);

            slewRate = Math.abs(gamepad1.right_trigger - gamepad1.left_trigger)*0.085;
            slewTarget = (gamepad1.left_trigger > 0 ? -maxTurnL : (gamepad1.right_trigger > 0 ? maxTurnR : robot.getCurrentTurretDegreePos() ));



            velocity += (gamepad1.dpad_up ? 10 : 0) - (gamepad1.dpad_down ? 10 : 0);
            if (gamepad1.xWasPressed()){
                velocity = preSetVelocity;
                angle = 0.69;
            } else if (gamepad1.bWasPressed()){
                velocity = 0;
            } else if (gamepad1.aWasPressed()){
                velocity = 800;
                angle = 0.4;
            }

            angle += (gamepad1.dpad_left ? 0.045 : 0) - (gamepad1.dpad_right ? 0.045 : 0);
            angle = Range.clip(angle, 0, 1); // 0.19

            robot.setShooterVelocity(velocity);
            robot.setHoodAngle(angle);

            if (gamepad1.yWasPressed()){
                if (intakeVel == -1000 || intakeVel == 1000){
                    intakeVel = 0;
                } else if (intakeVel == 0){
                    intakeVel = -1000;
                }
                robot.setIntakeSpeed(intakeVel); // 2800 max?
            } else if (gamepad1.dpad_down){
                intakeVel = 1000;
                robot.setIntakeSpeed(intakeVel);
            }


            if (gamepad2.a){
                spinAngle = 60;
            } else {
                spinAngle = 120;
            }

            if (!robot.spindexer.isBusy() && armClock > 10) {
                if (gamepad2.leftBumperWasPressed()) {
                    robot.setSpindexerRelativeAngle(spinAngle);
                } else if (gamepad2.rightBumperWasPressed()) {
                    robot.setSpindexerRelativeAngle(-spinAngle);
                }
            }

            if (gamepad2.b){
                armPos =  0.7;
                armClock = 0;
            }
            if (gamepad2.bWasReleased()){
                armPos = 1;
                armClock = 0;
            }

            if (gamepad2.x){
                robot.homeToAprilTag();
            } else {
                robot.setTurretPositionAbsolute(slewTarget, slewRate);
            }
            robot.setArmPos(armPos);


            // Telemetry for drivers + debugging
            telemetry.addData("Controls G2", "Left & Right Bumpers : Spindexer | B : Arm");
            telemetry.addData("Controls G1", "Left & Right Triggers : Turret " +
                    "\n | Left & Right Dpad : Hood Angle | Up & Down Dpad : Velocity");
            telemetry.addData("Inputs", "angle=%.2f   velocity=%.2f", angle, velocity);
            telemetry.addData("Measured Velocity: ", robot.shooter.getVelocity());
            telemetry.addData("Colors (RGB): ",  "R=%d%n G=%d%n B=%d%n ", robot.colorSensor.red(), robot.colorSensor.green(), robot.colorSensor.blue());
            // telemetry.addData("Heading(rad)", robot.getHeadingRadians()); / add a getter in RobotHardware if desired
            telemetry.update();

            // Pace loop-helps with readability and prevents spamming the DS
            sleep(50); // ~20 Hz;
            if (armClock <= 10){
                armClock++;
            }
        }
    }
}


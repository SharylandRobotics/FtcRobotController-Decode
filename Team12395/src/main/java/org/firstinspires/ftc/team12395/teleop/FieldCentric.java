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

import android.graphics.Color;
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
    public static double preSetAngleFar = 0.75;
    public static double preSetAngleClose = 0.4;

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

        double prevHeading = 0;
        double armClock = 9;
        int lastTrackingClock = 10;

        int autoShootClock = 0;

        boolean xToggle = false;
        boolean autoSense = false;


        robot.init();

        robot.limelight.start();

        waitForStart();


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
                angle = preSetAngleFar;
            } else if (gamepad1.bWasPressed()){
                velocity = 0;
            } else if (gamepad1.aWasPressed()){
                velocity = 800;
                angle = preSetAngleClose;
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
            } else if (gamepad1.dpadDownWasPressed()){
                if (intakeVel != 1000){
                    intakeVel = 1000;
                } else  {
                    intakeVel = 0;
                }

                robot.setIntakeSpeed(intakeVel);
            }

            if (gamepad1.dpadLeftWasPressed() ){
                autoSense = true;
            } else if (gamepad1.dpadRightWasPressed()){
                autoSense = false;
            }

            if (autoSense && !robot.spindexer.isBusy()){
                boolean done = robot.senseAutomaticSequence();
                if (done){
                    autoSense = false;
                }
            }

            // gamepad 2 --

            if (gamepad2.dpad_up){
                if (autoShootClock >= 0 && (robot.spindexerTarget % 120) == 0) {
                    robot.shootAutomaticSequence(autoShootClock);
                    autoShootClock++;
                    if (autoShootClock > 11) {
                        autoShootClock = 0;
                    }
                }
            } else {
                autoShootClock = 0;


                if (gamepad2.a) {
                    spinAngle = 60;
                } else {
                    spinAngle = 120;
                }

                if (!robot.spindexer.isBusy() && armClock > 8) {
                    if (gamepad2.leftBumperWasPressed()) { // ccw
                        robot.spindexerHandler((int) spinAngle);
                    } else if (gamepad2.rightBumperWasPressed()) { // cw
                        robot.spindexerHandler((int) -spinAngle);
                    }
                }

                if (gamepad2.b) {
                    armPos = 0.7;
                    armClock = 0;
                }
                if (gamepad2.bWasReleased()) {
                    armPos = 1;
                    armClock = 0;
                }

                robot.setArmPos(armPos);
            }

            if (autoShootClock < 0){
                autoShootClock++;
            }

            if (gamepad2.xWasPressed()){
                xToggle = !xToggle;
            }

            if (xToggle){
                double errorDeg = robot.homeToAprilTag();

                if (!Double.isNaN(errorDeg) ) {

                    double farFudge = 4;
                    if (velocity == preSetVelocity){
                        farFudge *= errorDeg/Math.abs(errorDeg);
                    }

                    robot.setTurretPositionRelative(errorDeg + (robot.getHeading() - prevHeading) + farFudge);
                    lastTrackingClock = 0;
                } else if (lastTrackingClock < 2000) {


                    robot.setTurretPositionRelative((robot.getHeading() - prevHeading));
                }else {
                    telemetry.addData("AprilTag Not Detected/Invalid ", "...");
                    robot.setTurretPositionRelative(slewTarget, slewRate);
                }
            } else {
                robot.setTurretPositionAbsolute(slewTarget, slewRate);
            }

            prevHeading = robot.getHeading();



            // Telemetry for drivers + debugging
            telemetry.addData("color?: ", robot.scanColor());
            telemetry.addData("busy?: ", robot.spindexer.isBusy());
            telemetry.addData("current Chamber: ", robot.chamber);
            telemetry.addData("current Mag: ", mag);
            telemetry.addData("current Pattern: ", pattern);
            telemetry.addData("Inputs", "angle=%.2f   velocity=%.2f", angle, velocity);
            telemetry.addData("Measured Velocity: ", robot.shooter.getVelocity());
            // telemetry.addData("Heading(rad)", robot.getHeadingRadians()); / add a getter in RobotHardware if desired
            telemetry.update();

            // Pace loop-helps with readability and prevents spamming the DS
            sleep(50); // ~20 Hz;
            if (armClock <= 8){
                armClock++;
            }

            if (lastTrackingClock <= 2000){
                lastTrackingClock++;
            }
        }
    }
}


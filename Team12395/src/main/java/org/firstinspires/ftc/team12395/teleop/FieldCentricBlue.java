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

@TeleOp(name="Field Centric (Blue)", group="TeleOp")
@Config
public class FieldCentricBlue extends LinearOpMode {

    // NOTE: One hardware instance per OpMode keeps mapping/IMU use simple and testable
    RobotHardware robot = new RobotHardware(this);

    public static double velocity = 0;
    public static double angle = 0.1;

    public static double preSetVelocityFar = 1440;
    public static double preSetAngleFar = 0.8;
    public static double preSetAngleClose = 0.4;
    public static double preSetVelocityClose = 1120;

    public static double armPos = 1;

    public static int intakeVel = 0;





    @Override
    public void runOpMode() {
        // Driver inputs (range roughly [-1, 1])
        double axial    ; // forward/back (+ forward)
        double lateral  ; // strafe left/right (+ right)
        double yaw      ; // rotation (+ CCW/left turn)

        double spinAngle;

        double prevHeading = 0;
        double armClock = 9;
        int lastTrackingClock = 10;
        double lastTargetTurretPos = 0;
        double lastTargetHeading = 0;

        int autoShootClock = 0;

        boolean xToggle = false;
        boolean autoSense = false;


        robot.init();

        robot.limelight.start();

        mag = "000";

        waitForStart();


        robot.disableDriveEncoders();


        // --- TELEOP LOOP ---
        while (opModeIsActive()) {

            axial   = -gamepad1.left_stick_y;
            lateral =  gamepad1.left_stick_x;
            yaw     =  gamepad1.right_stick_x;

            if (gamepad1.right_bumper){ axial *= 0.6; lateral *= 0.6; yaw *= 0.6; }
            robot.driveFieldCentric(axial, lateral, yaw);

            velocity += (gamepad1.dpad_up ? 10 : 0) - (gamepad1.dpad_down ? 10 : 0);
            if (gamepad1.xWasPressed()){
                velocity = preSetVelocityFar; angle = preSetAngleFar;
            } else if (gamepad1.bWasPressed()){
                velocity = 0;
            } else if (gamepad1.aWasPressed()){
                velocity = preSetVelocityClose; angle = preSetAngleClose;
            }

            angle += (gamepad1.dpad_left ? 0.045 : 0) - (gamepad1.dpad_right ? 0.045 : 0);
            angle = Range.clip(angle, 0, 1); // 0.19

            robot.setShooterVelocity(velocity);
            robot.setHoodAngle(angle);

            if (gamepad1.yWasPressed()){
                if (intakeVel == -1000 || intakeVel == 1000){
                    intakeVel = 0;

                } else if (intakeVel == 0){ intakeVel = -1000; }
                robot.setIntakeSpeed(intakeVel);
            } else if (gamepad1.dpadDownWasPressed()){
                if (intakeVel != 1000){
                    intakeVel = 1000;

                } else  { intakeVel = 0; }
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

            if (gamepad2.dpad_left) {
                if ( (robot.spindexerTarget % 120) != 0 && mag.charAt(chamber) == '0') {
                    robot.setChamberManual('P');
                }
            } else if (gamepad2.dpad_right) {
                if ( (robot.spindexerTarget % 120) != 0 && mag.charAt(chamber) == '0') {
                    robot.setChamberManual('G');
                }
            } else if (gamepad2.dpad_down && robot.solvePattern() != null){
                robot.spindexerHandler(120*robot.solvePattern()[0]);
            } else if (gamepad2.dpad_up){
                if (autoShootClock >= 0 && (robot.spindexerTarget % 120) == 0) {
                    robot.shootAutomaticSequence(autoShootClock);
                    autoShootClock++;
                }
            }
            if(!gamepad2.dpad_up) {
                if (autoShootClock != 0){
                    autoShootClock = 0;
                    armPos = 1; armClock = 0;
                }

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
                    armPos = 0.7; armClock = 0;
                }
                if (gamepad2.bWasReleased()) {
                    armPos = 1; armClock = 0;
                }
                robot.setArmPos(armPos);
            }

            if (autoShootClock > 11){ autoShootClock = 0;}

            if (gamepad2.xWasPressed()){
                xToggle = !xToggle;
                if (xToggle){
                    robot.playBeep("orb");
                } else {
                    robot.playBeep("orb_deep");
                }
            }

            if (xToggle){
                double errorDeg = robot.homeToAprilTagBlue();

                if (!Double.isNaN(errorDeg) ) {
                    double farFudge = 0;
                    if (velocity == preSetVelocityFar){ farFudge = Math.copySign(4, errorDeg); }

                    lastTargetTurretPos = errorDeg - 1.5*(robot.getHeading() - prevHeading) + farFudge;
                    robot.setTurretHandlerRelative(lastTargetTurretPos);
                    lastTargetTurretPos += robot.getCurrentTurretDegreePos();
                    lastTargetHeading = robot.getHeading();

                    lastTrackingClock = 0;
                } else if (lastTrackingClock < 2000) {
                    robot.setTurretHandlerAbsolute(lastTargetTurretPos + (robot.getHeading() - lastTargetHeading));
                } else {
                    robot.setTurretHandlerAbsolute(0);
                }
            } else {
                robot.setTurretHandlerAbsolute(0);
            }

            prevHeading = robot.getHeading();

            // Telemetry for drivers + debugging
            telemetry.addData("current Chamber: ", chamber);
            telemetry.addData("current Mag: ", mag);
            telemetry.addData("current Pattern: ", pattern);
            telemetry.addData("Inputs", "angle=%.2f   velocity=%.2f", angle, velocity);
            telemetry.addData("Measured Velocity: ", robot.shooter.getVelocity());
            telemetry.addData("turret running? ", robot.turretHandler.runToTarget());
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


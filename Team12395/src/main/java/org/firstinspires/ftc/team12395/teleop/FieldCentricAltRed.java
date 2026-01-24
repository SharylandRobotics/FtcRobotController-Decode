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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team12395.RobotHardware;

import static org.firstinspires.ftc.team12395.RobotHardware.*;

@TeleOp(name="Field Centric ALT (Red Solo)", group="TeleOp")
@Config
public class FieldCentricAltRed extends LinearOpMode {

    // NOTE: One hardware instance per OpMode keeps mapping/IMU use simple and testable
    RobotHardware robot = new RobotHardware(this);

    public static double velocity = 0;
    public static double angle = 0.1;

    public static double preSetVelocityFar = 1900;
    public static double preSetAngleFar = 0.3;
    public static double preSetAngleClose = 0.8;
    public static double preSetVelocityClose = 1400;

    public static int intakeVel = 0;

    @Override
    public void runOpMode() {
        // Driver inputs (range roughly [-1, 1])
        double axial    ; // forward/back (+ forward)
        double lateral  ; // strafe left/right (+ right)
        double yaw      ; // rotation (+ CCW/left turn)

        double tSkew = 0;

        double prevHeading = 0;
        int lastTrackingClock = 10;
        double lastTargetTurretPos = 0;
        double lastTargetHeading = 0;

        boolean turretToggle = false;
        boolean runTurnClock = false;

        int turnClock = 0;

        boolean runJammingClock = false;
        int jammingClock = 0;

        robot.init();

        robot.limelight.start();

        robot.mag = "000";

        robot.disableDriveEncoders();

        waitForStart();

        // --- TELEOP LOOP ---
        while (opModeIsActive()) {

            axial   = -gamepad1.left_stick_y*0.9;
            lateral =  gamepad1.left_stick_x*0.9;
            yaw     =  gamepad1.right_stick_x*0.9;

            if (gamepad1.right_trigger > 0.05){ axial *= 0.6; lateral *= 0.6; yaw *= 0.6; }
            robot.driveFieldCentric(axial, lateral, yaw);

            if (gamepad1.xWasPressed()){
                velocity = preSetVelocityFar; angle = preSetAngleFar;
            } else if (gamepad1.bWasPressed()){
                velocity = 0;
            } else if (gamepad1.aWasPressed()){
                velocity = preSetVelocityClose; angle = preSetAngleClose;
            }

            robot.setShooterVelocity(velocity);
            robot.setHoodAngle(angle);

            if (gamepad1.yWasPressed()){
                if (intakeVel == 1 || intakeVel == -1){
                    intakeVel = 0;

                } else if (intakeVel == 0){ intakeVel = 1; }
                robot.setIntakeSpeed(intakeVel*1600);
            } else if (gamepad1.dpadDownWasPressed()){
                if (intakeVel != -1){
                    intakeVel = - 1;

                } else  { intakeVel = 0; }
                robot.setIntakeSpeed(intakeVel*1600);
            }

            robot.scanColor();

            // gamepad 2 (g1 cuz solo)--
            if (!robot.spindexer.isBusy()) {
                if (gamepad1.leftBumperWasPressed()) { // ccw
                    robot.spindexerHandler(120);
                } else if (gamepad1.rightBumperWasPressed()) { // cw
                    if (velocity == preSetVelocityFar){
                        robot.spindexerHandler(-480, 500);
                    } else {
                        robot.spindexerHandler(-480);
                    }
                    robot.setMagManualBulk("000");
                } else if (false) {
                    robot.spindexerHandler(120*robot.solvePattern()[0]);
                } else if (!runTurnClock && false) {

                    if (scannedColor.equals(colorTypes.PURPLE)){
                        if (robot.mag.charAt(robot.chamber) == '0') {
                            robot.setChamberManual('P');
                            runTurnClock = true;
                        }
                    } else if (scannedColor.equals(colorTypes.GREEN)){
                        if (robot.mag.charAt(robot.chamber) == '0') {
                            robot.setChamberManual('G');
                            runTurnClock = true;
                        }
                    }
                }
            } else if (gamepad1.dpadLeftWasPressed()){
                robot.spindexer.setPower(0);
                runJammingClock = true;
            }

            if (runJammingClock && jammingClock < 3){
                turnClock = 0;
                runTurnClock = false;

                jammingClock++;
            } else {
                // dont run turn clock until done with jamming clock
                if (runTurnClock && turnClock < 3){
                    turnClock++;
                }

                if (turnClock == 2){
                    robot.spindexerHandler(120);
                    turnClock = 0;
                    runTurnClock = false;
                }
            }

            if (jammingClock == 2) {
                robot.maintainSpindexerHandler();
                jammingClock = 0;
                runJammingClock = false;
            }



            if (gamepad1.dpadRightWasPressed()){
                turretToggle = !turretToggle;
                if (turretToggle){
                    robot.playBeep("orb");
                } else {
                    robot.playBeep("orbDeep");
                }
            }

            if (turretToggle){
                double[] tData = robot.homeToAprilTagRed();
                double errorDeg = tData[0];

                if (!Double.isNaN(errorDeg) ) {
                    tSkew = tData[1];
                    double farFudge = 0;
                    if (velocity == preSetVelocityFar){ farFudge = Math.copySign(4, errorDeg); }

                    lastTargetTurretPos = errorDeg - 1*(robot.getHeading() - prevHeading) + farFudge;
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

            //robot.maintainSpindexerHandler();

            prevHeading = robot.getHeading();
            robot.turretHandler.runToTarget();

            if (gamepad2.a){
                robot.pattern = "GPP";
            } else if (gamepad2.b){
                robot.pattern = "PGP";
            } else if (gamepad2.y){
                robot.pattern = "PPG";
            }

            robot.getSpindexerOffset();

            // Telemetry for drivers + debugging
            telemetry.addData(robot.getMagPicture(), "");
            telemetry.addData("current Pattern: ", robot.pattern);
            telemetry.addData("Measured Velocity: ", robot.shooter.getVelocity());
            telemetry.addData("spindexer position? ", robot.getCurrentSpindexerDegreesPos() % 360);
            telemetry.addData("apt deg: ", tSkew);
            telemetry.update();

            // Pace loop-helps with readability and prevents spamming the DS
            sleep(50); // ~20 Hz;

            if (lastTrackingClock <= 2000){
                lastTrackingClock++;
            }
        }
    }
}


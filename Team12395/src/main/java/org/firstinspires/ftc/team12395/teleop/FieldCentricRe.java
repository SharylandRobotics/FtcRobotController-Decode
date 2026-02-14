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


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.team12395.RobotHardware;
import org.firstinspires.ftc.team12395.rr.Drawing;

import static org.firstinspires.ftc.team12395.RobotHardware.*;

@TeleOp(name="Field Centric Re (Blue Solo)", group="TeleOp")
@Config
public class FieldCentricRe extends LinearOpMode {

    // NOTE: One hardware instance per OpMode keeps mapping/IMU use simple and testable
    RobotHardware robot = new RobotHardware(this);

    public static double velocity = 0;
    public static double angle = 0.1;

    public static double preSetVelocityFar = 2000;
    public static double preSetAngleFar = 0.2;
    public static double preSetAngleClose = 0.8;
    public static double preSetVelocityClose = 1400;

    public static double intakeVel = 0;


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // Driver inputs (range roughly [-1, 1])
        double axial    ; // forward/back (+ forward)
        double lateral  ; // strafe left/right (+ right)
        double yaw      ; // rotation (+ CCW/left turn)

        int lastTrackingClock = 10;
        double lastTargetTurretPos = 0;
        double headingVelocity;

        boolean intakeToggle = false;
        boolean turretToggle = false;

        robot.init();

        robot.limelight.start();

        robot.mag = "000";

        robot.disableDriveEncoders();

        waitForStart();

        // --- TELEOP LOOP ---
        while (opModeIsActive()) {

            axial   = -gamepad1.left_stick_y;
            lateral =  gamepad1.left_stick_x;
            yaw     =  gamepad1.right_stick_x;

            robot.driveFieldCentric(axial, lateral, yaw);

            if (gamepad1.bWasPressed()){
                velocity = 0;
            } else if (gamepad1.aWasPressed()){
                velocity = preSetVelocityClose; angle = preSetAngleClose;
            }

            robot.setShooterVelocity(velocity);
            robot.setHoodAngle(angle);

            if (gamepad1.yWasPressed()){
                intakeToggle = !intakeToggle;
                intakeVel = Math.abs(intakeVel);
            } else if (gamepad1.dpadDownWasPressed()){
                if (!(intakeVel < 0) || !intakeToggle){
                    intakeToggle = true;
                    intakeVel = -intakeVel;
                } else {
                    intakeToggle = false;
                }
            }

            if (intakeToggle){
                robot.setIntakeSpeed(intakeVel);
            } else {
                robot.setIntakeSpeed(0);
            }



            // gamepad 2 (g1 cuz solo)--
            if (gamepad1.leftBumperWasPressed()) { // ccw
                robot.spindexerHandler(120);
            } else if (gamepad1.rightBumperWasPressed()) { // cw
                if (velocity == preSetVelocityFar){
                    robot.spindexerHandler(-480, 1000);
                } else {
                    robot.spindexerHandler(-480);
                }
                robot.setMagManualBulk("000");
            }
            if (gamepad1.dpadLeftWasPressed()){
                robot.spindexer.setVelocity(0);
            }


            if (gamepad1.dpadRightWasPressed()){
                turretToggle = !turretToggle;
            }

            robot.standardDrive.updatePoseEstimate();
            headingVelocity = robot.getHeadingVelocity();

            telemetry.addData("Heading Velocity: ",  headingVelocity);

            TelemetryPacket packet = new TelemetryPacket();

            // Limelight pose (blue) if valid
            Pose2d llpose = robot.fetchLocalizedPose(-90);

            if (!Double.isNaN(llpose.position.x) && !Double.isNaN(llpose.position.y)) {
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), llpose);

                if (!(headingVelocity > 80)) {
                    robot.setLocalizerPosition(llpose);
                }
            }

            Pose2d currentPose = robot.standardDrive.localizer.getPose();
            double angle = robot.turretAngleToTarget(new Vector2d(-65, -59), currentPose);
            telemetry.addData("Angle", Math.toDegrees(-angle));

            // Odometry pose (red) always
            packet.fieldOverlay().setStroke("#B53F51");
            Drawing.drawRobot(packet.fieldOverlay(), currentPose);

            packet.fieldOverlay().setStroke("#008000");
            Drawing.drawRobot(packet.fieldOverlay(), currentPose.plus(new Twist2d(new Vector2d(0,0), angle)));

            // Send once
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            if (turretToggle){
                robot.setTurretHandlerAbsolute(
                        Math.toDegrees(-angle)
                );
            } else {
                robot.setTurretHandlerAbsolute(0);
            }

            if (Math.abs(robot.spindexerFudge % 360) >= 3){
                robot.maintainSpindexerHandler();
            }

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
            telemetry.addData("spindexer heading? ", robot.getCurrentSpindexerDegreesPos() % 360);
            telemetry.addData("spindexerE heading? ", ((robot.spindexerE.getPositionAndVelocity().position/robot.spindexerETicksPerDegree) % 360));
            telemetry.addData("spindexer error: ", robot.spindexerFudge);
            telemetry.addData("turret deg: ", robot.getCurrentTurretDegreePos());
            telemetry.addData("target turret deg: ", lastTargetTurretPos);
            telemetry.addData("Distance to Target: ", robot.getDistanceFromTarget(new Vector2d(-65, -59), currentPose));



            telemetry.update();

            // Pace loop-helps with readability and prevents spamming the DS
            sleep(50); // ~20 Hz;

            if (lastTrackingClock <= 2000){
                lastTrackingClock++;
            }
        }

    }
}


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
        // Driver inputs (range roughly [-1, 1])
        double axial    = 0; // forward/back (+ forward)
        double lateral  = 0; // strafe left/right (+ right)
        double yaw      = 0; // rotation (+ CCW/left turn)

        boolean servoOn = false;
        boolean lastServoState = false;

        boolean shooterOn = false;
        boolean lastShooterState = false;

        boolean intakeOn = false;
        boolean lastIntakeState = false;

        boolean intakeServoOn = false;
        boolean lastIntakeServoState = false;
        // --- INIT PHASE ---
        // WHY: Centralized init in RobotHardware sets motor directions, encoder modes, IMU orientation, etc.
        // TODO(STUDENTS): Confirm IMU orientation & Motor names in RobotHardware.init()
        robot.init();

        while(opModeInInit()) {

            // Student Note: Pre‑start check — rotate robot by hand; heading should change.
            // "Vision: Ready (AprilTag)" means camera + processor initialized.
            telemetry.addData("Status", "Hardware Initialized");
            telemetry.addData("Vision", "Ready (AprilTag)");
            telemetry.addData("Mode", "INIT");
//            telemetry.addData("Obelisk", robot.hasObeliskMotif() ? String.format("%s (ID %s)",
//                    robot.getObeliskMotif(), robot.getObeliskTagId()) : "–");
            telemetry.addData("Heading", "%4.0f", robot.getHeading());
            telemetry.update();
        }
        // Wait for driver to press START on Driver Station
        waitForStart();
        if (isStopRequested()) return;
        // --- TELEOP LOOP ---
        while (opModeIsActive()) {
            //shooting mechanism

            boolean currentShooterState = (gamepad1.right_trigger > .5);
            if(currentShooterState && !lastShooterState){
                shooterOn = !shooterOn;
            }
            if(shooterOn){
                robot.turretPower(1);
            }
            else{
                robot.turretPower(0);
            }
            lastShooterState = currentShooterState;

            boolean currentIntakeState = (gamepad1.left_trigger > .5);

            if(currentIntakeState && !lastIntakeState){
                intakeOn = !intakeOn;
            }
            if(intakeOn){
                robot.intakePower(-.5);
            }
            else{
                robot.intakePower(0);
            }
            lastIntakeState = currentIntakeState;


            //hood servo

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

            // intake servo

            boolean currentIntakeServoState = (gamepad1.aWasPressed());
            if(currentIntakeServoState && !lastIntakeServoState){
                intakeServoOn = !intakeServoOn;
            }
            if(intakeServoOn){
                robot.setIntakeServo(0);
            }
            else{
                robot.setIntakeServo(1);
            }
            lastIntakeServoState = currentIntakeServoState;


            // Keep vision fresh before using pose values each loop
            robot.updateAprilTagDetections();

            // Student Note: Hold LB for precision (slow) mode.
            boolean slow = gamepad1.left_bumper;
            double scale = slow ? 0.4 : 1.0;

            axial = -gamepad1.left_stick_y * scale;
            lateral = gamepad1.left_stick_x * scale;
            yaw = gamepad1.right_stick_x * scale;

            // --- Vision helpers for concise telemetry ---
            Integer goalId = robot.getGoalTagId();
            double range = robot.getGoalRangeIn();
            double bearing = robot.getGoalBearingDeg();
            double elevation = robot.getGoalElevationDeg();

            // Approx horizontal distance and aim-above-horizontal (camera pitched up 15°)
            double horiz = (Double.isNaN(range) || Double.isNaN(bearing))
                    ? Double.NaN
                    : range * Math.cos(Math.toRadians(bearing));
            double aimAboveHorizontal = (Double.isNaN(elevation) ? Double.NaN : (15.0 + elevation));

            // Driver Assist: hold RB to auto-drive toward the visible goal tag (range->drive, yaw->strafe, bearing->turn).
            boolean autoAssist = gamepad1.right_bumper;
            boolean didAuto = false;
            if (autoAssist) {
                robot.updateAprilTagDetections();
                didAuto = robot.autoDriveToGoalStep();
            }

            // Student Note: Field‑centric drive call (mixing happens in RobotHardware) unless auto applied.
            if (!didAuto) {
                robot.driveRobotCentric(axial, lateral, yaw);
            }

            telemetry.addData("Mode", slow ? "SLOW" : "NORMAL");
            telemetry.addData("Assist", autoAssist ? (didAuto ? "AUTO→TAG" : "NO TAG") : "MANUAL");
            telemetry.addData("Heading", "%4.0f°", robot.getHeading());
            telemetry.addData("Drive", "ax=%.2f  lat=%.2f  yaw=%.2f", axial, lateral, yaw);

            String motif = robot.hasObeliskMotif() ? String.format("%s (ID %s)", robot.getObeliskMotif(), robot.getObeliskTagId()) : "–";

            telemetry.addData("Pose", "rng=%.1f in  brg=%.1f°  elev=%.1f°", range, bearing, elevation);
            telemetry.addData("Aim",  "horiz=%.1f in  aboveHoriz=%s",
                    horiz,
                    Double.isNaN(aimAboveHorizontal) ? "–" : String.format("%.1f°", aimAboveHorizontal));
            telemetry.addData("TagYaw", "%.1f°", robot.getTagYawDeg());
            // non tag code
            telemetry.addData("Controls", "Drive/Strafe: Left Stick | Turn: Right Stick");
            telemetry.addData("Inputs", "axial=%.2f   lateral=%.2f   yaw=%.2f", axial, lateral, yaw);
            // Optional: expose heading during tuning
            // telemetry.addData("Heading(rad)", robot.getHeadingRadians()); / add a getter in RobotHardware if desired
            //servo stuff
            telemetry.addData("Drive", "Left Stick");
            telemetry.addData("Turn", "Right Stick");
            //telemetry.addData("intake Servo Up/Down", "Button Y toggle");


            telemetry.addData("-", "-------");
            telemetry.addData("Obelisk", motif);
            telemetry.addData("Goal", (goalId != null) ? goalId : "–");

            telemetry.addData("Intake Servo Up/Down", "Button A toggle");
            telemetry.addData("Hood Servo Up/Down", "Button B toggle");

            telemetry.addData("Turret Motor", "Right Trigger toggle");
            telemetry.addData("Intake Motor", "Left Trigger toggle");

            telemetry.addData("Drive to tag", "Hold Right Bumper");
            telemetry.addData(" Slow drive to tag", "Hold Left Bumper");





            telemetry.update();

            sleep(50);


        }
    }
}


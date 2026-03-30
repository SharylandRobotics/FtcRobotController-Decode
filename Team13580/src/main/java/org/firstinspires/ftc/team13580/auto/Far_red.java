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

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team13580.RobotHardware;
import org.firstinspires.ftc.team13580.roadrunner.MecanumDrive;

import java.lang.Math;

@Autonomous(name = "Red far", group = "auto")

// Autonomous routine using gyro-based driving with RobotHardware helpers
public class Far_red extends LinearOpMode {

    // Instantiate RobotHardware and link this OpMode
    RobotHardware robot = new RobotHardware(this);


    class SpinUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            robot.setIntakePower(0.5);
            return false;
        }
    }

    class spinUp2 implements Action {
        private double power;
        public spinUp2(double power){
            this.power = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            robot.setIntakePower(power);
            return false;
        }


    }
    class Shoot implements Action {
        private int vel;

        public Shoot(int vel){
            this.vel = vel;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            robot.setOuttakeVelocity(vel);
            return false;
        }
    }

    class RightKicker implements Action {
        private double power;
        public RightKicker(double power){
            this.power = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            robot.setKickerPower(power);
            return false;
        }

    }

    class LeftKicker implements Action {
        private double power;
        public LeftKicker(double power){
            this.power = power;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            robot.setKickerLeftPower(power);
            return false;
        }
    }

    Action spinUp(){
        return new SpinUp();
    }
    Action shoot(int vel) {
        return new Shoot(vel);
    }

    Action rightkick(double power){
        return new RightKicker(power);
    }
    Action leftkick(double power){
        return new LeftKicker(power);
    }
    Action spinUp2(double power){
        return new spinUp2(power);
    }




    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap,
                new Pose2d(62,15, Math.toRadians(180)));
        // Initialize all motors and IMU before start
        robot.init();

        Action path1 = drive.actionBuilder(new Pose2d(62,15, Math.toRadians(180)))
                .setTangent(Math.atan2(0, 50-62))
                .lineToXLinearHeading(50,Math.toRadians(157),new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                .build();

        Action path2 = drive.actionBuilder(new Pose2d(50,15, Math.toRadians(157)))
                .setTangent(Math.atan2(24-15, 41-50))
                .lineToYLinearHeading(24,Math.toRadians(90),new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                .setTangent(Math.atan2(60-24, 0))
                .lineToYLinearHeading(63,Math.toRadians(90),new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                .build();

        Action path3 = drive.actionBuilder(new Pose2d(41,63, Math.toRadians(90)))
                .setTangent(Math.atan2(15-63, 50-41))
                .lineToYLinearHeading(15,Math.toRadians(152),new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                .build();

        Action path4_First_gate_intake = drive.actionBuilder(new Pose2d(50,15, Math.toRadians(152)))
                .setTangent(Math.atan2(60-15, 62-50))
                .lineToYLinearHeading(60,Math.toRadians(90),new TranslationalVelConstraint(90), new ProfileAccelConstraint(-60, 100))
                .setTangent(Math.atan2(15-60, 50-62))
                .lineToYLinearHeading(15,Math.toRadians(170),new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                .build();

        Action path5 = drive.actionBuilder(new Pose2d(50,15, Math.toRadians(170)))
                .setTangent(Math.atan2(60-15,65-50))
                .lineToYLinearHeading(60,Math.toRadians(90),new TranslationalVelConstraint(90), new ProfileAccelConstraint(-60, 100))
                .setTangent(Math.atan2(15-60,50-65))
                .lineToYLinearHeading(12,Math.toRadians(158),new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                .build();

        Action path6 = drive.actionBuilder(new Pose2d(50,12, Math.toRadians(158)))
                .setTangent(Math.atan2(60-12, 65-50))
                .lineToYLinearHeading(60,Math.toRadians(90),new TranslationalVelConstraint(95), new ProfileAccelConstraint(-60, 100))
                .setTangent(Math.atan2(15-60, 50-65))
                .lineToYLinearHeading(15,Math.toRadians(162),new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                .build();

        Action path7 = drive.actionBuilder(new Pose2d(50,15, Math.toRadians(162)))
                .setTangent(Math.atan2(60-15, 68-50))
                .lineToYLinearHeading(64,Math.toRadians(90),new TranslationalVelConstraint(95), new ProfileAccelConstraint(-60, 100))
                .setTangent(Math.atan2(15-64, 50-68))
                .lineToYLinearHeading(15,Math.toRadians(166),new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                .build();

        Action path8 = drive.actionBuilder(new Pose2d(50,15, Math.toRadians(166)))
                .setTangent(Math.atan2(60-15, 68-50))
                .lineToYLinearHeading(63,Math.toRadians(90),new TranslationalVelConstraint(90), new ProfileAccelConstraint(-60, 100))
                .setTangent(Math.atan2(15-63, 50-68))
                .lineToYLinearHeading(15,Math.toRadians(165),new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                .build();

        Action path9 = drive.actionBuilder(new Pose2d(50,15, Math.toRadians(165)))
                .setTangent(Math.atan2(40-15, 40-50))
                .lineToYLinearHeading(40,Math.toRadians(90),new TranslationalVelConstraint(100), new ProfileAccelConstraint(-60, 100))
                .build();



        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                path1,
                                shoot(1450)
                        ),
                        new SleepAction(2),
                        spinUp2(.8),
                        rightkick(-1),
                        leftkick(-1),
                        new SleepAction(.7),
                        rightkick(.3),
                        leftkick(.3),
                        new ParallelAction(
                                path2,
                                shoot(1500)
                        ),
                        new ParallelAction(
                                path3,
                                spinUp2(.5)
                        ),
                        shoot(1510),
                        spinUp2(.8),
                        rightkick(-1),
                        leftkick(-1),
                        new SleepAction(.7),
                        rightkick(.3),
                        leftkick(.3),
                        //intake 3 from wall
                        path4_First_gate_intake,
                        rightkick(-1),
                        leftkick(-1),
                        new SleepAction(.7),
                        rightkick(.3),
                        leftkick(.3),
                        //imaginarry 1st
                        path5,
                        rightkick(-1),
                        leftkick(-1),
                        new SleepAction(.7),
                        rightkick(.3),
                        leftkick(.3),
                        //imaginarry 2nd
                        path6,
                        shoot(1510),
                        rightkick(-1),
                        leftkick(-1),
                        new SleepAction(.7),
                        rightkick(.3),
                        leftkick(.3),
                        //imaginarry 3rd
                        path7,
                        shoot(1510),
                        rightkick(-1),
                        leftkick(-1),
                        new SleepAction(.7),
                        rightkick(.3),
                        leftkick(.3),
                        //imaginary 4th
                        path8,
                        shoot(1510),
                        rightkick(-1),
                        leftkick(-1),
                        new SleepAction(.7),
                        rightkick(.3),
                        leftkick(.3),
                        new ParallelAction(
                                path9,
                                spinUp2(0),
                                shoot(0),
                                rightkick(0),
                                leftkick(0)
                        )




                )
        );
    }
}

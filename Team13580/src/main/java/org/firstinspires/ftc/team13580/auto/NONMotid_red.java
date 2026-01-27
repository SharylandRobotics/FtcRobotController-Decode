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

@Autonomous(name = "Motid red", group = "auto")

// Autonomous routine using gyro-based driving with RobotHardware helpers
public class NONMotid_red extends LinearOpMode {

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
                new Pose2d(-50,50, Math.toRadians(126)));
        // Initialize all motors and IMU before start
        robot.init();

        Action path1 = drive.actionBuilder(new Pose2d(-50,50, Math.toRadians(126)))
                .setTangent(Math.atan2(24-50, -30+50))
                .lineToY(24)
                .build();

        Action path2 = drive.actionBuilder(new Pose2d(-30,24, Math.toRadians(126)))
                .setTangent(Math.atan2(26-24, 18+30))
                .lineToYLinearHeading(26, Math.toRadians(90))
                .setTangent(Math.atan2(30-26, 0))
                .lineToYLinearHeading(30, Math.toRadians(90))
                .setTangent(Math.atan2(0, 11-18))
                .lineToX(11)
                .setTangent(Math.atan2(54-34, 0))
                .lineToYLinearHeading(54, Math.toRadians(90))
                .build();


        Action path3_opengate = drive.actionBuilder(new Pose2d(11,54, Math.toRadians(90)))
                .setTangent(Math.atan2(28-54, 0))
                .lineToYLinearHeading(28, Math.toRadians(90))
                .setTangent(Math.atan2(24-28, -30-11))
                .lineToYLinearHeading(24, Math.toRadians(175))
                .turnTo(Math.toRadians(130))
                .build();

        Action path4_intakemiddle = drive.actionBuilder(new Pose2d(-30,24, Math.toRadians(130)))
                .setTangent(Math.atan2(28-24, -10+30))
                .lineToYLinearHeading(28, Math.toRadians(90))
                .setTangent(Math.atan2(38-28, 0))
                .lineToYLinearHeading(38, Math.toRadians(90))
                .setTangent(Math.atan2(0, -10+10))
                .lineToX(-14)
                .setTangent(Math.atan2(53-38, 0))
                .lineToYLinearHeading(53, Math.toRadians(90))
                .build();

        Action path5_shoot3rd = drive.actionBuilder(new Pose2d(-14,53, Math.toRadians(90)))
                .setTangent(Math.atan2(24-53, -30+14))
                .lineToYLinearHeading(24, Math.toRadians(130))
                        .build();

        Action path6_intakelast = drive.actionBuilder(new Pose2d(-30,24, Math.toRadians(130)))
                .setTangent(Math.atan2(28-24, 38+30))
                .lineToYLinearHeading(28, Math.toRadians(90))
                .setTangent(Math.atan2(34-28, 0))
                .lineToYLinearHeading(34, Math.toRadians(90))
                .setTangent(Math.atan2(0, 34-38))
                .lineToX(34)
                .setTangent(Math.atan2(56-34, 0))
                .lineToYLinearHeading(56, Math.toRadians(90))
                        .build();

        Action path7_lastshot = drive.actionBuilder(new Pose2d(34,56, Math.toRadians(90)))
                .setTangent(Math.atan2(24-56, -30-34))
                .lineToYLinearHeading(24, Math.toRadians(130))
                        .build();

        Action path8_park = drive.actionBuilder(new Pose2d(-30,24, Math.toRadians(130)))
                .setTangent(Math.atan2(40-24, -10+30))
                .lineToYLinearHeading(40, Math.toRadians(90), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-60, 100))
                        .build();







        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                path1,
                                shoot(1150)
                        ),
                        new SleepAction(.5),
                        spinUp(),
                        rightkick(1),
                        leftkick(1),
                        new SleepAction(2),
                        rightkick(0),
                        leftkick(0),
                        new ParallelAction(
                                path2,
                                spinUp2(.5)
                        ),
                        new ParallelAction(
                                path3_opengate,
                                shoot(1150),
                                spinUp2(.5)
                        ),
                        spinUp2(.5),
                        rightkick(1),
                        leftkick(1),
                        new SleepAction(2),
                        rightkick(0),
                        leftkick(0),
                        new ParallelAction(
                                path4_intakemiddle,
                                spinUp2(.5)
                        ),
                        new ParallelAction(
                                path5_shoot3rd,
                                spinUp2(.5),
                                shoot(1150)
                        ),
                        rightkick(1),
                        leftkick(1),
                        new SleepAction(2),
                        rightkick(0),
                        leftkick(0),
                        new ParallelAction(
                                path6_intakelast,
                                spinUp2(.5)
                        ),
                        spinUp2(.3),
                        new ParallelAction(
                                path7_lastshot,
                                shoot(1150)
                        ),
                        rightkick(1),
                        leftkick(1),
                        spinUp2(.5),
                        new SleepAction(2),
                        new ParallelAction(
                                path8_park,
                                rightkick(0),
                                leftkick(0),
                                spinUp2(0),
                                shoot(0)
                        )

                )
        );
    }
}

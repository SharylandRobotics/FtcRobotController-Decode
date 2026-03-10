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

@Autonomous(name = "Blue close", group = "auto")

// Autonomous routine using gyro-based driving with RobotHardware helpers
public class NONMotid_Blue extends LinearOpMode {

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
                new Pose2d(-49,-51, Math.toRadians(-126)));
        // Initialize all motors and IMU before start
        robot.init();

        Action path1 = drive.actionBuilder(new Pose2d(-49,-51, Math.toRadians(-126)))
                .setTangent(Math.atan2(-24+51, -30+49))
                .lineToYLinearHeading(-24, Math.toRadians(-130))
                .build();

        Action path2 = drive.actionBuilder(new Pose2d(-30,-24, Math.toRadians(-130)))
                .setTangent(Math.atan2(-20+24, 11+30))
                .lineToYLinearHeading(-20, Math.toRadians(-90))
                .setTangent(Math.atan2(-62+20, 0))
                .lineToYLinearHeading(-62, Math.toRadians(-90), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-60, 100))
                .setTangent(Math.atan2(-20+62, 0))
                .lineToYLinearHeading(-20, Math.toRadians(-90), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-60, 100))
                .build();

        Action path3_opengate = drive.actionBuilder(new Pose2d(11,-20, Math.toRadians(-90)))
                .setTangent(Math.atan2(-18+20, -20-11))
                .lineToYLinearHeading(-18, Math.toRadians(-140), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-60, 100))
                .build();

        Action path4_First_gate_intake = drive.actionBuilder(new Pose2d(-20,-18, Math.toRadians(-140)))
                .setTangent(Math.atan2(-20+18, 10.5+20))
                .lineToYLinearHeading(-20, Math.toRadians(-115))
                .setTangent(Math.atan2(-60+20, 0))
                .lineToYLinearHeading(-60, Math.toRadians(-115))
                .build();

        Action slide_to_intake = drive.actionBuilder(new Pose2d(10.5,-60, Math.toRadians(-115)))
                .setTangent(Math.atan2(0, 19-10.5))
                .lineToX(19, new TranslationalVelConstraint(60), new ProfileAccelConstraint(-60, 100))

                .build();

        Action path5 = drive.actionBuilder(new Pose2d(19,-60, Math.toRadians(-115)))
                .setTangent(Math.atan2(-22+58, 10.5-19))
                .lineToYLinearHeading(-22, Math.toRadians(-90), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-60, 100))
                .setTangent(Math.atan2(-18+22, -22-10.5))
                .lineToYLinearHeading(-18, Math.toRadians(-140), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                .build();

        Action path6_second_gate_intake = drive.actionBuilder(new Pose2d(-22,-18, Math.toRadians(-140)))
                .setTangent(Math.atan2(-20+18, 12+22))
                .lineToYLinearHeading(-20, Math.toRadians(-115))
                .setTangent(Math.atan2(-58+20, 0))
                .lineToYLinearHeading(-58, Math.toRadians(-115), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                .setTangent(Math.atan2(0, 17-12))
                .lineToX(17, new TranslationalVelConstraint(60), new ProfileAccelConstraint(-60, 100))
                .build();

        Action path7 = drive.actionBuilder(new Pose2d(17,-58, Math.toRadians(-115)))
                .setTangent(Math.atan2(-20+58, 0))
                .lineToYLinearHeading(-20, Math.toRadians(115), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-60, 100))
                .setTangent(Math.atan2(-24+20, -22-17))
                .lineToYLinearHeading(-24, Math.toRadians(140), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                .build();

        Action path8_close_intake = drive.actionBuilder(new Pose2d(-22,-24, Math.toRadians(-145)))
                .setTangent(Math.atan2(-18+24, -12+22))
                .lineToYLinearHeading(-18, Math.toRadians(-90))
                .setTangent(Math.atan2(-53+18, 0))
                .lineToYLinearHeading(-53, Math.toRadians(-90), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-60, 100))
                .build();

        Action path9 = drive.actionBuilder(new Pose2d(-12,-53, Math.toRadians(-90)))
                .setTangent(Math.atan2(-20+53, -16+12))
                .lineToYLinearHeading(-20, Math.toRadians(-130), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-60, 100))
                .build();

        Action path10_park = drive.actionBuilder(new Pose2d(-16,-20, Math.toRadians(-130)))
                .setTangent(Math.atan2(-49+20, -14+16))
                .lineToYLinearHeading(-49, Math.toRadians(-90), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-60, 100))
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
                        spinUp2(.8),
                        rightkick(-1),
                        leftkick(-1),
                        new SleepAction(1.5),
                        rightkick(.3),
                        leftkick(.3),
                        new ParallelAction(
                                path2,
                                spinUp2(.5)
                        ),
                        new ParallelAction(
                                path3_opengate,
                                spinUp2(.9),
                                shoot (1150)
                        ),
                        rightkick(-1),
                        leftkick(-1),
                        new SleepAction(1.5),
                        rightkick(.3),
                        leftkick(.3),
                        spinUp2(.3),
                        path4_First_gate_intake,
                        spinUp2(.9),
                        shoot(1150),
                        new SleepAction(.3),
                        slide_to_intake,
                        new SleepAction(1),
                        path5,
                        new ParallelAction(
                                rightkick(-1),
                                leftkick(-1),
                                spinUp2(.9)
                        ),
                        new SleepAction(1.5),
                        rightkick(.3),
                        leftkick(.3),
                        spinUp2(.6),
                        path8_close_intake,
                        spinUp2(.6),
                        path9,
                        shoot(1200),
                        new ParallelAction(
                                rightkick(-1),
                                leftkick(-1),
                                spinUp2(.8)
                        ),
                        new SleepAction(1.5),
                        new ParallelAction(
                                path10_park,
                                rightkick(-1),
                                leftkick(-1),
                                spinUp2(.8),
                                shoot(0)
                        )









                )
        );
    }
}
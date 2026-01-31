package org.firstinspires.ftc.team13581.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team13581.RobotHardware;

@Autonomous(name="Robot Centric Shoot Blue Team", group="Autonomous")

public class RobotCentricShootBlue extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode(){

        final double AXIAL_SPEED       = 0.5;
        final double LATERAL_SPEED     = 0.5;
        final double YAW_SPEED         = 0.3;
        robot.init();

        while(opModeInInit()) {
            // Display heading and status continuously during init loop
            telemetry.addData("Status", "Hardware Initialized");
            telemetry.addData("Heading", "%4.0f", robot.getHeading());
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        if (opModeIsActive()) {
            robot.setShootSpeed(1300);
            robot.setAimPos(0.75);
            //robot.setTurretPos(0.5);
            sleep(1000);
            robot.turnToHeading(YAW_SPEED, 0);
            robot.autoRobotCentric(AXIAL_SPEED, -33.0, 0.0);
            sleep(100);
            robot.setIntake1(1);
            robot.setIntake2(.7);
            sleep(2500);
            robot.setIntake1(0);
            robot.setIntake2(0);
            robot.setShootSpeed(0);
            sleep(1000);
            robot.turnToHeading(YAW_SPEED, -50.0);
            robot.holdHeading(YAW_SPEED, -50.0, 0.5);
            robot.autoRobotCentric(AXIAL_SPEED, -19.5, -50.0);// move more back
            robot.turnToHeading(YAW_SPEED, 40-2.5);
            robot.holdHeading(YAW_SPEED, 40-2.5, 0.5);

            robot.setIntake1(1);
            robot.setIntake2(.5);
            robot.autoRobotCentric((AXIAL_SPEED-0.25), 15, 40);// move to balls
            sleep(300);
            robot.setIntake2(0);
            robot.autoRobotCentric((AXIAL_SPEED-0.25), 9, 40);// move to balls
            //robot.autoRobotCentric((AXIAL_SPEED-0.25), 35.0, -46);// move to balls
            robot.setIntake1(0);
            robot.autoRobotCentric(AXIAL_SPEED, -35.0, 40);// move back from balls


            robot.setShootSpeed(1500);
            sleep(1000);
            robot.setAimPos(0.45);
            robot.turnToHeading(YAW_SPEED, 0); // turn to shoot
            robot.holdHeading(YAW_SPEED, 0, 0.5);
            robot.setIntake1(-0.2);
            sleep(200);
            robot.setIntake1(1);
            robot.setIntake2(.7);
            sleep(200);
            robot.setIntake1(0);
            robot.setIntake2(0);
            sleep(1000);
            robot.setIntake1(1);
            robot.setIntake2(.7);
            sleep(1200);
            robot.setIntake1(0);
            robot.setIntake2(0);
            robot.setBackPower(0);
            sleep(1000);
            robot.turnToHeading(YAW_SPEED, -50.0);
            robot.holdHeading(YAW_SPEED, -50.0, 0.5);

            robot.autoRobotCentric(AXIAL_SPEED+2, -24.0, -50.0);

            robot.turnToHeading(YAW_SPEED, -130.0);
            robot.holdHeading(YAW_SPEED, -130.0, 0.5);

            sleep(1000);

            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);  // Pause to display last telemetry message.
        }
    }
}
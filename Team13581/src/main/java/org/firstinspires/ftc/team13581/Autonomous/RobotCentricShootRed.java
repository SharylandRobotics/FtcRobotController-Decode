package org.firstinspires.ftc.team13581.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team13581.RobotHardware;

@Autonomous(name="Robot Centric Shoot Red Team", group="Autonomous")

public class RobotCentricShootRed extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode(){

        final double AXIAL_SPEED       = 0.5;
        final double LATERAL_SPEED     = 0.5;
        final double YAW_SPEED         = 0.3;
        final double GOAL_DEG = 0.0;
        final double SIDE_WAYS = 50.0;
        final double AWAY = -40.0+2.5;
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
            robot.turnToHeading(YAW_SPEED, GOAL_DEG);
            robot.autoRobotCentric(AXIAL_SPEED, -33.0, GOAL_DEG);
            sleep(100);
            robot.setIntake1(1);
            robot.setIntake2(.7);
            sleep(2500);
            robot.setIntake1(0);
            robot.setIntake2(0);
            robot.setShootSpeed(0);
            sleep(1000);
            robot.turnToHeading(YAW_SPEED, SIDE_WAYS);
            robot.holdHeading(YAW_SPEED, SIDE_WAYS, 0.5);
            robot.autoRobotCentric(AXIAL_SPEED, -19.5, SIDE_WAYS);// move more back
            robot.turnToHeading(YAW_SPEED, AWAY);
            robot.holdHeading(YAW_SPEED, AWAY, 0.5);

            robot.setIntake1(1);
            robot.setIntake2(.5);
            robot.autoRobotCentric((AXIAL_SPEED-0.25), 15, AWAY);// move to balls
            sleep(300);
            robot.setIntake2(0);
            robot.autoRobotCentric((AXIAL_SPEED-0.25), 9, AWAY);// move to balls
            //robot.autoRobotCentric((AXIAL_SPEED-0.25), 35.0, -46);// move to balls
            robot.setIntake1(0);
            robot.autoRobotCentric(AXIAL_SPEED, -35.0, AWAY);// move back from balls
            robot.turnToHeading(YAW_SPEED, SIDE_WAYS);
            robot.holdHeading(YAW_SPEED, SIDE_WAYS, 0.5);
            robot.autoRobotCentric(AXIAL_SPEED, 11.5, SIDE_WAYS);// move more forward

            robot.setShootSpeed(1300);
            sleep(1000);
            robot.setAimPos(0.45);
            robot.turnToHeading(YAW_SPEED, GOAL_DEG); // turn to shoot
            robot.holdHeading(YAW_SPEED, GOAL_DEG, 0.5);
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
            robot.turnToHeading(YAW_SPEED, SIDE_WAYS);
            robot.holdHeading(YAW_SPEED, SIDE_WAYS, 0.5);

            robot.autoRobotCentric(0.9, -34.0, SIDE_WAYS);

            robot.turnToHeading(YAW_SPEED, AWAY);
            robot.holdHeading(YAW_SPEED, AWAY, 0.5);
            robot.autoRobotCentric(0.9, 10.0, AWAY);

            sleep(1000);

            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);  // Pause to display last telemetry message.
        }
    }
}
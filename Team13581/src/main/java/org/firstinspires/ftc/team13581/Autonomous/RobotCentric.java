package org.firstinspires.ftc.team13581.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team13581.RobotHardware;

@Autonomous(name="Robot Centric Auto", group="Autonomous")

public class RobotCentric extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode(){

        final double AXIAL_SPEED       = 0.4;
        final double LATERAL_SPEED     = 0.4;
        final double YAW_SPEED         = 0.2;
        robot.init();

        waitForStart();

        while(opModeIsActive()) {

            robot.autoRobotCentric(AXIAL_SPEED, -48.0, 0.0);
            robot.setBackPower(1);
            robot.setAimPos(0.55);
            sleep(1000);
            robot.setFrontPower(1);
            sleep(1000);
            robot.setLeverPos(0.4);
            sleep(1000);
            robot.setLeverPos(0.17);
            robot.setFrontPower(0);
            robot.setBackPower(0);
            robot.setAimPos(0.2);
            break;
        }
    }
}

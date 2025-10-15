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
        double speed = AXIAL_SPEED;
        robot.init();

        waitForStart();

        while(opModeIsActive()) {

            robot.autoRobotCentric(speed, 48.0, 0.0);

            break;
        }
    }
}

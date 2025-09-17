package org.firstinspires.ftc.team00000.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team00000.RobotHardware;

/**
 * Simple example autonomous using Robot-centric helpers.
 *
 * Pattern:
 *  – call init(), waitForStart(), start()
 *  – sequence autoRobotCentric(...) and turn/hold helpers
 *  – use idle() inside any loops (RobotHardware already does this)
 *  – stop() at end or on early exit
 *
 *  NOTE: for events, consider:
 *  – adding a global timeout guard per segment
 *  – guarding on isStopRequested()
 *  – providing alliance/starting pose parameters via @Config or gamepad
 */
@Autonomous(name="Robot Centric Auto", group="Auto")

public class RobotCentricAuto extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware(this);

    // Example tunables for move/turn speeds
    private static final double AXIAL_SPEED = 0.5;
    private static final double YAW_SPEED   = 0.5;

    @Override
    public void runOpMode(){
        robot.init();

        telemetry.addLine("Auto ready");
        telemetry.update();

        waitForStart();
        robot.start();

        if (opModeIsActive() && !isStopRequested()) {
            // Example path segments (values are placeholders-adjust to your game plan)
            robot.autoRobotCentric(48.0, AXIAL_SPEED, 0.0); // forward 48", hold heading 0°
            robot.turnToHeading(90.0, YAW_SPEED); // turn to 90°
            robot.holdHeading(90.0, 0.5); // settle briefly

            robot.autoRobotCentric(17.0, AXIAL_SPEED, 90.0); // forward 17", hold 90°
            robot.turnToHeading(-45.0, YAW_SPEED);
            robot.holdHeading(-45.0, 0.5);

            robot.autoRobotCentric(17.0, AXIAL_SPEED, -45.0);
            robot.turnToHeading(45.0, YAW_SPEED);
            robot.holdHeading(45.0, 0.5);

            robot.autoRobotCentric(17.0, AXIAL_SPEED, 45.0);
            robot.turnToHeading(0.0, YAW_SPEED);
            robot.holdHeading(0.0, 1.0);

            robot.autoRobotCentric(-48.0, AXIAL_SPEED, 0.0); // return/back up
        }
    }
}

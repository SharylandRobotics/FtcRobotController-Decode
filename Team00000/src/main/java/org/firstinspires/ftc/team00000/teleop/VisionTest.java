package org.firstinspires.ftc.team00000.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team00000.RobotHardware;

@TeleOp(name="Vision Test", group="Debug")
public class VisionTest extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        robot.init();
        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Vision debug mode: AprilTag detection");
        telemetry.update();

        waitForStart();
        robot.start();

        int heartbeat = 0;

        while (opModeIsActive() && !isStopRequested()) {
            robot.periodic();

            // Vision telemetry
            robot.telemetryVision(telemetry); // helper prints minimal tag info

            telemetry.addData("HasTagFix", robot.hasTagFix());
            telemetry.addData("Detections", (int) robot.getFieldPose().yM);
            telemetry.addData("TagID", robot.getFieldPose().tagId);
            telemetry.addData("Range (m)", "%.2f", robot.getRangeToGoalM());
            telemetry.addData("Heading (blended)", "%.1f", robot.getHeading());
            telemetry.addData("Heading (raw vision)", "%.1f", robot.getFieldPose().headingDeg);

            heartbeat++;
            telemetry.addData("Heartbeat", heartbeat);

            telemetry.update();
            idle();
        }
        robot.stop();
    }
}

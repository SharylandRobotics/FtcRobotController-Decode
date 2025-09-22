package org.firstinspires.ftc.team00000.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team00000.RobotHardware;

// TeleOp mode for testing AprilTag vision telemetry
@TeleOp(name="Vision Debug", group="zDebug")
public class VisionTest extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        // Initialize hardware and vision
        robot.init();
        telemetry.setMsTransmissionInterval(50);

        waitForStart();
        robot.start();

        // Update vision and push telemetry each cycle
        while (opModeIsActive() && !isStopRequested()) {
            robot.periodic();

            // Compute horizontal bearing from tag translation
            boolean fix = robot.hasTagFix();
            double tx = robot.getFieldPose().x;
            double ty = robot.getFieldPose().y;
            double tz = robot.getFieldPose().z;
            double lateralHeading = (!Double.isNaN(tx) && !Double.isNaN(ty) && (tx != 0 || ty != 0))
                    ? Math.toDegrees(Math.atan2(tx, ty))
                    : Double.NaN;
            double elevationAngle = (!Double.isNaN(tz) && !Double.isNaN(ty) && (tz != 0 || ty != 0))
                    ? Math.toDegrees(Math.atan2(tz, ty))
                    : Double.NaN;

            // Report vision state and pose values
            telemetry.addData("HasTagFix", fix);
            telemetry.addData("TagID", robot.getFieldPose().tagId);
            telemetry.addData("Range (m)", "%.2f", robot.getRangeToGoalM());
            telemetry.addData("Lateral Heading (deg)", "%.1f", lateralHeading);
            telemetry.addData("Elevation Angle (deg)", "%.1f", elevationAngle);
            telemetry.addData("Tx (m)", "%.2f", tx);
            telemetry.addData("Tz (m)", "%.2f", tz);
            telemetry.update();
            idle();
        }
        robot.stop();
    }
}

package org.firstinspires.ftc.team12395;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class rrActions {
    RobotHardware robot;
    LinearOpMode myOpMode;
    public rrActions(RobotHardware robot, LinearOpMode myOpMode){
        this.robot = robot;
        this.myOpMode = myOpMode;
    }

    // spindexer object class
    public class Spindexer {
        private DcMotorEx spindexer;

        public Spindexer(){
            spindexer = myOpMode.hardwareMap.get(DcMotorEx.class, "spindexer");

            spindexer.setDirection(DcMotorEx.Direction.FORWARD);

            spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        public class spindexerHandler implements Action{
            private int deg;

            public spindexerHandler(int deg){
                this.deg = deg;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                robot.spindexerHandler(deg);
                return false;
            }
        }

        public Action spindexerTargetAdd(int deg){
            return new spindexerHandler(deg);
        }

        public Action sortSpindexer(){
            return new spindexerHandler(120*robot.solvePattern()[0]);
        }
    }

    public class liftArm {
        // class vars
        private Servo xArm;

        // class constructor & hardware mapper
        public liftArm(){
            xArm = myOpMode.hardwareMap.get(Servo.class, "xArm");
        }

        // actual action class/ do-er
        public class liftArmToPosition implements Action {
            private double pos; // in percentage (0 to 1)
            public liftArmToPosition(double pos){
                this.pos = pos;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.setArmPos(pos);
                return false;
            }

        }

        // usable method/ action class shortcut
        public Action liftArmUp(){
            return new liftArmToPosition(0.7);
        }

        public Action liftArmDown(){
            return new liftArmToPosition(1);
        }
    }

    public class Turret{
        private servoDrivenEncoder turretHandler;

        public Turret(){
            turretHandler = new servoDrivenEncoder(robot.turretE, robot.turretR, robot.turretL);
        }

        public class turnTurretRelative implements Action{
            private double deg;

            public turnTurretRelative(double deg){
                this.deg = deg;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                robot.setTurretHandlerRelative(deg);

                return !turretHandler.runToTarget(); // you are not done?
            }
        }

        public class turnTurretAbsolute implements Action{
            private double deg;

            public turnTurretAbsolute(double deg){
                this.deg = deg;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                robot.setTurretHandlerAbsolute(deg);

                return !turretHandler.runToTarget(); // you are not done?
            }
        }

        public Action turnTurretBy(double deg){
            return new turnTurretRelative(deg);
        }

        public Action turnTurretTo(double deg){
            return new turnTurretAbsolute(deg);
        }
    }

    public class Shooter{
        private DcMotorEx shooter;

        public Shooter(){
            shooter = myOpMode.hardwareMap.get(DcMotorEx.class, "shooter");
            shooter.setDirection(DcMotorEx.Direction.REVERSE);
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public class setShooterVelocity implements Action{
            private int vel;

            public setShooterVelocity(int vel){
                this.vel = vel;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                robot.setShooterVelocity(vel);
                return false;
            }
        }

        public Action setShooterVel(int vel){
            return new setShooterVelocity(vel);
        }
    }

    public class Hood{
        private Servo hoodAngle;

        public Hood(){
            hoodAngle = myOpMode.hardwareMap.get(Servo.class, "hood_angle");
        }

        public class setHoodAngle implements Action{
            private double pos;

            public setHoodAngle(double pos){
                this.pos = pos;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                robot.setHoodAngle(pos);
                return false;
            }
        }

        public Action setHoodAngle(double pos){
            return new setHoodAngle(pos);
        }
    }

    public class Intake{
        private DcMotorEx intake;
        public Intake(){
            intake = myOpMode.hardwareMap.get(DcMotorEx.class, "intake");
            intake.setDirection(DcMotorEx.Direction.FORWARD);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        public class setIntakeVelocity implements Action{
            private int vel;
            public setIntakeVelocity(int vel){
                this.vel = vel;
            }

            @Override
            public boolean run(TelemetryPacket packet){
                robot.setIntakeSpeed(vel);
                return false;
            }
        }

        public Action setIntakeVel(int vel){
            return new setIntakeVelocity(vel);
        }
    }

    public class LimeLight{
        private Limelight3A limelight;
        public LimeLight(){
            limelight = myOpMode.hardwareMap.get(Limelight3A.class, "limelight-rfc");
        }

        public class processObelisk implements Action{
            public processObelisk(){
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                return !robot.processObelisk();
            }
        }

        public class processBlueGoal implements Action {
            public processBlueGoal(){
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                robot.homeToAprilTagBlue();
                return false;
            }
        }

        public class processRedGoal implements Action {
            public processRedGoal(){
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                robot.homeToAprilTagRed();
                return false;
            }
        }

        public Action scanForObelisk(){
            return new processObelisk();
        }

        public Action scanForBlue(){
            return new processBlueGoal();
        }
        public Action scanForRed(){
            return new processRedGoal();
        }

    }
}

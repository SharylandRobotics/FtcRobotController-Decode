package org.firstinspires.ftc.team12395;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.atomic.AtomicReferenceFieldUpdater;

@Config
public class servoDrivenEncoder {
    PIDController controller = new PIDController(P, I, D);

    OverflowEncoder encoder;
    CRServo servo1, servo2;
    int targetPos;
    int currentPos;
    int lastPos;
    double output = 0;

    double dt = 0;
    double lastTimeStamp = 0;

    public static double P = 0.0003;
    public static double I = 0;
    public static double D = 0;

    private double maxVel = RobotHardware.spindexerTicksPerDegree*60;
    private double maxAccel = maxVel*5;
    private double currentVel = 0;

    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxVel, maxAccel);
    TrapezoidProfile.State goal = new TrapezoidProfile.State(targetPos, 0);
    TrapezoidProfile.State current = new TrapezoidProfile.State(currentPos, 0);
    TrapezoidProfile profile = new TrapezoidProfile(constraints, goal, current);

    TrapezoidProfile.State setpoint;

    public servoDrivenEncoder(OverflowEncoder encoder, CRServo servo1, CRServo servo2){
        controller.reset();

        this.encoder = encoder;
        this.servo1 = servo1;
        this.servo2 = servo2;

        this.servo1.setDirection(servo1.getDirection());
        this.servo2.setDirection(servo2.getDirection());

        controller.setTolerance(5);
    }

    public boolean runToPosition(int targetTicks){
        targetPos = targetTicks;


        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0){ lastTimeStamp = currentTimeStamp;}
        dt = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        setpoint = profile.calculate(dt);

        currentPos = getCurrentPosition();
        currentVel = (currentPos  - lastPos) / dt;
        lastPos = currentPos;

        output = controller.calculate(currentVel, setpoint.velocity);
        setServoPowers(output/ maxVel);

        if (Math.abs(currentPos - targetPos) < 6){
            stopServos();
            return true;
        } else {
            return false;
        }
    }

    public void setPGain(){
        controller.setP(P);
    }
    public void setIGain(){
        controller.setI(I);
    }


    public void setDGain(){
        controller.setD(D);
    }

    private void setServoPowers(double p){

        p = Range.clip(p, -1, 1);
        servo1.setPower(p);
        servo2.setPower(p);
    }

    private void stopServos(){
        servo1.setPower(0);
        servo2.setPower(0);
    }

    public double getServoPower(){
        return servo1.getPower();
    }

    public int getCurrentPosition(){
        return encoder.getPositionAndVelocity().position;
    }

    public double getCurrentError(){
        return controller.getPositionError();
    }

    public double getOutput(){
        return output;
    }

}

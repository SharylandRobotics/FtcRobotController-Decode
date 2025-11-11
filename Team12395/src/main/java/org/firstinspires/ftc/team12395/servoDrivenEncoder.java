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


    OverflowEncoder encoder;
    CRServo servo1, servo2;
    int currentPos;
    double output = 0;
    int targetPos;


    public static double P = 0.00015;
    public static double I = 0.000001;
    public static double D = 0.0000042;

    PIDController controller = new PIDController(P, I, D);

    public servoDrivenEncoder(OverflowEncoder encoder, CRServo servo1, CRServo servo2){
        controller.reset();

        this.encoder = encoder;
        this.servo1 = servo1;
        this.servo2 = servo2;

        this.servo1.setDirection(servo1.getDirection());
        this.servo2.setDirection(servo2.getDirection());

        controller.setTolerance(10);
        controller.setSetPoint(0);
        targetPos = 0;
    }

    public boolean runToPosition(int targetTicks){
        currentPos = getCurrentPosition();
        controller.setSetPoint(targetTicks);
        if (!controller.atSetPoint()){
            output = controller.calculate(currentPos);
            setServoPowers(output);
            return false;
        } else {
            stopServos();
            return true;
        }
    }

    public void setTargetPos(int target){
        controller.setSetPoint(target);
        targetPos = target;
    }

    public boolean runToTarget(){
        currentPos = getCurrentPosition();
        output = controller.calculate(currentPos, targetPos);
        if (!controller.atSetPoint()){
            setServoPowers(output);
            return false;
        } else {
            stopServos();
            return true;
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
        //p = Range.clip(Math.abs(p), 0, 1)*(p/Math.abs(p));
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

    public int getSetPos(){
        return (int) controller.getSetPoint();
    }

    public boolean getAtPoint(){
        return controller.atSetPoint();
    }

    public double[] getPID(){
        return controller.getCoefficients();
    }

}

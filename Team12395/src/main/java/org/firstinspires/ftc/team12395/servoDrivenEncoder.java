package org.firstinspires.ftc.team12395;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.arcrobotics.ftclib.controller.PDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

@Config
public class servoDrivenEncoder {
    PDController controller = new PDController(P,D);

    OverflowEncoder encoder;
    CRServo servo1, servo2;
    int targetPos;
    int currentPos;
    int prevPos;
    byte atTargetCount;

    public static double P = 0.01;
    public static double D = 0;

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
        currentPos = getCurrentPosition();
        controller.setSetPoint(targetTicks);
        if (!controller.atSetPoint()){
            setServoPowers(controller.calculate(currentPos));
            return false;
        } else {
            stopServos();
            return true;
        }
    }

    public void setPGain(){
        controller.setP(P);
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

}

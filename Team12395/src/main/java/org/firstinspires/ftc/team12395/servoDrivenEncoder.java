package org.firstinspires.ftc.team12395;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

@Config
public class servoDrivenEncoder {

    OverflowEncoder encoder;
    CRServo servo1, servo2;
    int target;
    int currentPos;

    public static double P = 0.001;

    public servoDrivenEncoder(OverflowEncoder encoder, CRServo servo1, CRServo servo2){
        this.encoder = encoder;
        this.servo1 = servo1;
        this.servo2 = servo2;

        this.servo1.setDirection(servo1.getDirection());
        this.servo2.setDirection(servo2.getDirection());
    }

    public void runToPosition(int targetTicks){
        target = targetTicks;
        currentPos = getCurrentPosition();
        if (!isAtTarget()) {
            activeTargetController();
        }
    }

    public boolean isAtTarget(){
        return currentPos == target;
    }

    private void activeTargetController(){
        int error = currentPos - target;
        if (error > 0){
            setServoPowers(-Math.abs(error)); // ccw

        } else if (error < 0){
            setServoPowers(Math.abs(error)); // cw
        }
    }

    private void setServoPowers(double p){
        p *= P;
        p = Range.clip(p, -1, 1);
        servo1.setPower(p);
        servo2.setPower(p);
    }

    public double getServoPower(){
        return servo1.getPower();
    }

    public int getCurrentPosition(){
        return encoder.getPositionAndVelocity().position;
    }

}

package org.firstinspires.ftc.team12395;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

public class servoDrivenEncoder {

    DcMotorEx encoder;
    CRServo servo1, servo2;
    int target;
    int currentPos;

    public servoDrivenEncoder(DcMotorEx encoder, CRServo servo1, CRServo servo2){
        this.encoder = encoder;
        this.servo1 = servo1;
        this.servo2 = servo2;

        this.servo1.setDirection(servo1.getDirection());
        this.servo2.setDirection(servo2.getDirection());
    }

    public void runToPosition(int targetTicks){
        target = targetTicks;
        currentPos = encoder.getCurrentPosition();
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
            setServoPowers(-Math.abs(Range.clip(error, -1, 1))); // ccw

        } else if (error < 0){
            setServoPowers( Math.abs(Range.clip(error, -1, 1))); // cw
        }
    }

    private void setServoPowers(double p){
        servo1.setPower(p);
        servo2.setPower(p);
    }

}

package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TestBenchServo {
    private Servo servoPos;
    private CRServo servoRot;// More Descriptive Names ususally

    public void init(HardwareMap hwMap){
        servoPos = hwMap.get(Servo.class, "servo");
        servoRot = hwMap.get(CRServo.class, "CRservo");
        servoPos.scaleRange(0.5,1);//Setting Range from Midpoint to 180
        servoPos.setDirection(Servo.Direction.REVERSE);
        servoRot.setDirection(CRServo.Direction.REVERSE);

    }
    public void setServoPos (double angle){
        servoPos.setPosition(angle);
    }
    public void setServoRot (double power){
        servoRot.setPower(power);
    }

}

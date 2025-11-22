package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArcadeDrive {
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    public void init(HardwareMap hwMap){
        frontLeftMotor = hwMap.get(DcMotor.class,"front_left_motor");
        frontRightMotor = hwMap.get(DcMotor.class,"front_right_motor");
        backLeftMotor = hwMap.get(DcMotor.class,"back_left_motor");
        backRightMotor = hwMap.get(DcMotor.class,"front_right_motor");

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
    }
    public void drive(double throttle, double spin) {
        double LeftPower = throttle + spin;
        double rightPower = throttle - spin;
        double largest = Math.max(Math.abs(LeftPower), Math.abs(rightPower));
        if (largest > 1.0) {
            LeftPower /= largest;
            rightPower /= largest;
        }
        frontLeftMotor.setPower(LeftPower);
        frontRightMotor.setPower(rightPower);
        backLeftMotor.setPower(LeftPower);
        backRightMotor.setPower(rightPower);
    }
}

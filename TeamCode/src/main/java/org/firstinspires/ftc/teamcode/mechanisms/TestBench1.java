package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestBench1 {
    private DigitalChannel touchSensor; //Descriptive name is usually recommended
    private DcMotor motor; //More specific name when typically used
    private double ticksPerRev;// revolution
    public void init(HardwareMap hwMap){
        //Touch Sensor Code when we actually use it
        touchSensor = hwMap.get(DigitalChannel.class, "Touch_Sensor");//This is filler. Need to match what is written in the config file on the controller hub.
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        //Dc Motor Code
        motor = hwMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ticksPerRev = motor.getMotorType().getTicksPerRev();
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    //Touch Sensor:
    public boolean getTouchSensorState(){
        return !touchSensor.getState();
    }
    public boolean isTouchSensorReleased(){
        return touchSensor.getState();
    }
    //Motor
    public void setMotorSpeed(double speed){
        motor.setPower(speed);

    }
    public double getMotorRevs(){
        return motor.getCurrentPosition() / ticksPerRev; // normalizings ticks to revolutions 2:1 ratio
    }
    public void setMotorZeroBehaviour (DcMotor.ZeroPowerBehavior zeroBehaviour){
        motor.setZeroPowerBehavior(zeroBehaviour);
    }
}

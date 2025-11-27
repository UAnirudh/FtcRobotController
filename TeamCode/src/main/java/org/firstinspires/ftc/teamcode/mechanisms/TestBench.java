package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestBench {
    private DigitalChannel touchSensor; //Descriptive name is usually recommended

    public void init(HardwareMap hwMap){
        touchSensor = hwMap.get(DigitalChannel.class, "Touch_Sensor");//This is filler. Need to match what is written in the config file on the controller hub.
        touchSensor.setMode(DigitalChannel.Mode.INPUT);

    }

    public boolean getTouchSensorState(){
        return !touchSensor.getState();
    }
    public boolean isTouchSensorReleased(){
        return touchSensor.getState();
    }
}

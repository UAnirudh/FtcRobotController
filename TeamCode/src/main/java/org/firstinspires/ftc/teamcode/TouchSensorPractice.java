package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.TestBench;

@TeleOp
public class TouchSensorPractice extends OpMode {
    TestBench bench = new TestBench();
    @Override
    public void init(){
        bench.init(hardwareMap);
    }

    @Override
    public void loop() {
        String sensorState = "Not Pressed!";
        if (bench.getTouchSensorState()){
            sensorState = "pressed!";
        }
        telemetry.addData("The sensor state is ",sensorState);
    }
}

package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestBenchColor {
    NormalizedColorSensor colorSensor;

    public enum DetectedColor{
        RED,
        BLUE,
        Yellow,
        UNKNOWN
    }
    public void init(HardwareMap hwMap){
        colorSensor = hwMap.get(NormalizedColorSensor.class, "Color_Sensor");
        colorSensor.setGain(4);
    }
    public DetectedColor getDetectedColor(Telemetry telemetry){
        NormalizedRGBA colors = colorSensor.getNormalizedColors(); // return 4 values

        float normRed, normGreen, normBlue;
        normRed = colors.red/ colors.alpha;
        normGreen = colors.green/ colors.alpha;
        normBlue = colors.blue/ colors.alpha;

        telemetry.addData("Red", normRed);
        telemetry.addData("Green", normGreen);
        telemetry.addData("Blue", normBlue);
        // TODO add if statements for specific
        /* This is just filler data need s to be calibrated.
        red, green, blue
        RED = >.35, <. 3, <. 3
        YELLOW = >.5, >.9, <.6
        BLUE = <. 2, < .5, > .5
        */

        if (normRed > 0.35 && normGreen < 0.3 && normBlue < 0.3) {
            return DetectedColor.RED;
        }
        else if (normRed > 0.5 && normGreen > 0.9 && normBlue < 0.6) {
            return DetectedColor.YELLOW;
        }
        else if (normRed < 0.2 && normGreen < 0.5 && normBlue > 0.5) {
            return DetectedColor.BLUE;
        }
        
        else{
            return DetectedColor.UNKNOWN;
        }


    }
}

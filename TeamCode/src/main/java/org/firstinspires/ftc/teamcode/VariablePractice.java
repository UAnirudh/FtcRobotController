package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp
public class VariablePractice extends OpMode {
    @Override
    public void init(){
        int teamNumber = 11138;
        double motorSpeed = 0.75;
        boolean clawClosed = true;
        String teamName = "Robo Eclipse";
        int motorAngle = 90;

        telemetry.addData("Team Number: ", teamNumber);
        telemetry.addData("Motor Speed: ", motorSpeed);
        telemetry.addData("Is the Claw Closed: ", clawClosed);
        telemetry.addData("My name is ", teamName);
        telemetry.addData("Motor angle is ",motorAngle);
    }
    @Override
    public void loop(){

    }



}

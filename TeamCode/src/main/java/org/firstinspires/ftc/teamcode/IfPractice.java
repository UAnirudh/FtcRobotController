package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp
public class IfPractice extends OpMode {
    @Override
    public void init(){
        
    }
    @Override
    public void loop(){
        /*
        boolean a_button = gamepad1.a; //Press is True, depress is false
        if (a_button){
            telemetry.addData("A button ", "pressed");
        }
        else{
            telemetry.addData("The A button","is not pressed");
        }
        telemetry.addData("Is the A button pressed: ", a_button);

         */
//        if (leftY < 0){
//            telemetry.addData("Left stick is ", "NEGATIVE");
//
//        }
//        else if (leftY > 0.5){
//            telemetry.addData("Left stick is ", "more than 50% power");
//        }
//        else if (leftY > 0) {
//            telemetry.addData("Left stick is ", "POSITIVE");
//        }
//        else {
//         telemetry.addData("Left stick is ", "0");
//        }
//        double leftY = gamepad1.left_stick_y;
//        if (LeftY < 0.1 && leftY > -0.1) {
//            telemetry.addData("Left Stick", "In Dead Zone");
//        }
//        telemetry.addData("Left Stick Value is ",leftY);
        double motorSpeed = gamepad1.left_stick_y;
        if (!gamepad1.a) {
            motorSpeed *= 0.5;

            telemetry.addData("Left Stick value", motorSpeed);
        }

    }
}



/*

AND - && if (leftY < 0.5 && leftY > 0) {
OR || if (LeftY < 0 || rightY < 0) {
NOT ! if (!clawClose) {
*/

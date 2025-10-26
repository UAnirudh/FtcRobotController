package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp
public class GamePadPractice extends OpMode {
    public void init(){

    }
    public void loop(){
        // runs 50x a second
        double triggerSum = gamepad1.left_trigger+gamepad1.right_trigger;
        double speedForward = -gamepad1.left_stick_y/2.0;
        double differenceOfx = gamepad1.left_stick_x-gamepad1.right_stick_x;
        telemetry.addData("x",gamepad1.left_stick_x);
        telemetry.addData("y",speedForward);
        telemetry.addData("a button",gamepad1.a);
        telemetry.addData("X",gamepad1.right_stick_x);
        telemetry.addData("Y",gamepad1.right_stick_y);
        telemetry.addData("b button",gamepad1.b);
        telemetry.addData("Difference of 'x's",differenceOfx);
        telemetry.addData("Sum of triggers",triggerSum);
    }
}

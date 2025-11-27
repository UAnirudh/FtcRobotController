package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.TestBench1;

@TeleOp
public class DcMotorPractice extends OpMode {
    double motorSpeed = gamepad1.left_stick_y;

    TestBench1 bench = new TestBench1();
    @Override
    public void init() {
        bench.init(hardwareMap);
    }

    @Override
    public void loop() {
        /*
        if (bench.getTouchSensorState()) {
            bench.setMotorSpeed(0.5);
        }
         */
        if (gamepad1.a){
            bench.setMotorZeroBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else if (gamepad1.b){
            bench.setMotorZeroBehaviour(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        bench.setMotorSpeed(motorSpeed);

        telemetry.addData("Motor Revs", bench.getMotorRevs());
    }

}

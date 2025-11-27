package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp
public class UseRobotLocationOpMode extends OpMode {
    RobotLocationPractice robotLocationPractice = new RobotLocationPractice(0);

    @Override
    public void init() {
        robotLocationPractice.setAngle(0);
        robotLocationPractice.setX(0);
        robotLocationPractice.setY(0);
    }
    @Override
    public void loop() {
        // A button and B button
        if(gamepad1.a){
            robotLocationPractice.turnRobot(0.1);
        }
        else if(gamepad1.b){
            robotLocationPractice.turnRobot(-0.1);
        }
        // Gamepad Left and Right

        if(gamepad1.dpad_left){
            robotLocationPractice.changeX(0.1);
        }
        else if(gamepad1.dpad_right){
            robotLocationPractice.changeX(-0.1);
        }
        // Gamepad Up and Down
        if(gamepad1.dpad_up){
            robotLocationPractice.changeY(0.1);
        }
        else if(gamepad1.dpad_down){
            robotLocationPractice.changeY(-0.1);
        }
        telemetry.addData("Heading", robotLocationPractice.getHeading());
        telemetry.addData("Angle", robotLocationPractice.getAngle());
        telemetry.addData("X value",robotLocationPractice.getX());
        telemetry.addData("Y value",robotLocationPractice.getY());
    }
}

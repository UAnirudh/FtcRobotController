package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class RumbleTest extends OpMode {
//    boolean wasA, isA;
    double endGameStart;
    boolean isEndGame;
    
    @Override
    public void init() {


    }
    public void start(){
        endGameStart = getRuntime() + 100;
    }
    
    
    @Override
    public void loop() {
        /*isA = gamepad1.a;
        if (isA && wasA){
            //gamepad1.rumble(100);
            //gamepad1.rumbleBlips(3);
            gamepad1.rumble( 1.0, 0, 100);
        }
        wasA = isA;*/

        // End Game check
        if (getRuntime() >= endGameStart&& !isEndGame) {
            gamepad1.rumbleBlips(3);
            isEndGame = true;
        }
    }
}

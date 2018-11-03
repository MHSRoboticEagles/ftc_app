package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto Depot Side", group ="Robot15173")
public class AutoDepotSide extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        runAutoMode();
    }

    @Override
    protected void act() {
        super.act();
        robot.encoderLift(0.5, 1,0, telemetry);
        move(0.8, 51);

//        for(int i = 0; i < 99; i++){
//            robot.rotateScoop(i/100, telemetry);
//        }
//        runtime.reset();
//
//        while(runtime.seconds() <= 5){
//            robot.rotateScoop(0.9, telemetry);
//        }

        runtime.reset();
        while (runtime.seconds() <2){
            robot.moveArmUp(0.4, telemetry);
        }

        move(0.8, -33);

        robot.encoderPivot(0.5, 5, 0, telemetry);

        move(0.8, 40);

        robot.encoderPivot(0.5, 3.5, 0, telemetry);

        move(0.8, 55);



    }
}

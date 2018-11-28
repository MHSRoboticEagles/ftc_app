package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Auto Depot Side", group ="Robot15173")
@Disabled
public class AutoDepotSide extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        runAutoMode();
    }

    @Override
    protected void act() {
        super.act();
        if (shouldRaiseLift) {
            robot.encoderLift(0.5, 3, 0, telemetry);
        }
        move(DRIVE_SPEED, 51);

        runtime.reset();
        robot.rotateScoop(0.9, telemetry);
        while (runtime.seconds() <= 2){

        }
        robot.stopScoop();

        move(DRIVE_SPEED, -33);

        robot.encoderPivot(PIVOT_SPEED, 5, 0, telemetry);

        move(DRIVE_SPEED, 40);

        robot.encoderPivot(PIVOT_SPEED, 3.5, 0, telemetry);

//        raiseArm();

        move(DRIVE_SPEED, 25);



    }
}

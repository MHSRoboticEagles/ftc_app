package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Crater Ground", group ="Robot15173")
public class AutoCraterSide extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        runAutoMode();
    }

    @Override
    protected void act() {
        robot.encoderLift(0.8, 6,0, telemetry);
        move(DRIVE_SPEED, 1);
        //find gold
        goldPosition = findGold(-1,false);

        runToTargetShort();
    }
}

package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Depot Ground Short", group ="Robot15173")
@Disabled
public class AutoDepotSideShort extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        runAutoMode();
    }

    @Override
    protected void act() {
        robot.encoderLift(0.8, 7,0, telemetry);
        move(DRIVE_SPEED, 2);
        //find gold
        goldPosition = findGold(-1, false);

        runToTargetShort();
    }
}

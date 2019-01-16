package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.skills.GoldPosition;

@Autonomous(name="Depot Ground", group ="Robot15173")
public class AutoDepotSide extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        runAutoMode();
    }

    @Override
    protected void act() {
        robot.encoderLift(0.8, 6,0, telemetry);
        move(DRIVE_SPEED, 1);
        //find gold
        goldPosition = findGold(-1, false);

        runToTarget();
    }
}

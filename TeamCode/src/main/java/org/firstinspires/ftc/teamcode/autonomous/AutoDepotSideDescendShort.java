package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Depot Short", group ="Robot15173")
public class AutoDepotSideDescendShort extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        runAutoMode();
    }

    @Override
    protected void act() {
        shouldRaiseLift = false;
        descend();
        //find gold
        goldPosition = findGold(-1, false);

        runToTargetShort();
    }
}

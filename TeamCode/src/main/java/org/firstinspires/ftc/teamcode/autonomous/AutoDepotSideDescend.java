package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Lift Depot ", group ="Robot15173")
public class AutoDepotSideDescend extends AutoDepotSide {
    @Override
    public void runOpMode() throws InterruptedException {
        runAutoMode();
    }

    @Override
    protected void act() {
        shouldRaiseLift = false;
        descend();
        robot.encoderLift(0.5, -3, 0, telemetry);
        super.act();
    }
}

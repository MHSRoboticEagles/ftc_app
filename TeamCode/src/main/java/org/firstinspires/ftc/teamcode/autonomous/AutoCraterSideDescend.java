package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Lift Crater", group ="Robot15173")
@Disabled
public class AutoCraterSideDescend extends AutoCraterSide {
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

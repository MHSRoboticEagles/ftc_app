package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto Crater Simple", group ="Robot15173")
public class AutoCraterSide extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        runAutoMode();
    }

    @Override
    protected void act() {
        super.act();
        robot.encoderLift(0.5, 1,0, telemetry);
        move(0.8, 51);

    }
}

package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="TF Test Right View", group ="Robot15173")
public class TFtestRight extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        runAutoMode();
    }

    @Override
    protected void act() {
        super.act();
        findGold(1);

    }
}

package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.gamefield.MineralLineUp;
import org.firstinspires.ftc.teamcode.skills.MineralDetection;

@Autonomous(name="TF Test Left View", group ="Robot15173")
public class TFtest extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        runAutoMode();
    }

    @Override
    protected void act() {
        super.act();
        findGold(-1);
        while (opModeIsActive()){
            //do nothing
        }
    }
}
package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto Crater Side", group ="Robot15173")
public class AutoCraterSide extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        runAutoMode();
    }

    @Override
    protected void crawl() {
        robot.encoderDrive(DRIVE_SPEED, 48, 48, 0, telemetry);
    }

    @Override
    protected void turn() {
//        turnLeft(robot.TURN_90);
    }

    @Override
    protected void moveToCrater() {
        robot.encoderStrafe(DRIVE_SPEED/4, -24.0, 0, telemetry);
    }
}

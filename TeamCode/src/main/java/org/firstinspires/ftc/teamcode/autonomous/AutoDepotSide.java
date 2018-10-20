package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto Depot Side", group ="Robot15173")
public class AutoDepotSide extends AutoBase {
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
        turnRight(robot.TURN_90);
    }

    @Override
    protected void moveToCrater() {
        robot.moveArmUp(0.4, telemetry);
        robot.encoderDrive(DRIVE_SPEED, 30, 30, 0, telemetry);
    }
}

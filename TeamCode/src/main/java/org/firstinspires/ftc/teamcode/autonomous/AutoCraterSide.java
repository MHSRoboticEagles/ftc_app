package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Auto Crater Simple", group ="Robot15173")
@Disabled
public class AutoCraterSide extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        runAutoMode();
    }

    @Override
    protected void act() {
        super.act();
        if (shouldRaiseLift) {
            robot.encoderLift(0.7, 3, 0, telemetry);
        }
//        raiseArm();
        move(DRIVE_SPEED, 40);

    }
}

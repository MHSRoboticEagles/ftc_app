package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto Depot Lite", group ="Robot15173")
public class AutoDepotLite extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        runAutoMode();
    }

    @Override
    protected void act() {
        shouldRaiseLift = false;
        descend();
        robot.encoderLift(0.5, -3, 0, telemetry);
        if (shouldRaiseLift) {
            robot.encoderLift(0.5, 3, 0, telemetry);
        }
        move(DRIVE_SPEED, 51);

        move(DRIVE_SPEED, -33);

        robot.encoderPivot(PIVOT_SPEED, 5, 0, telemetry);


    }
}

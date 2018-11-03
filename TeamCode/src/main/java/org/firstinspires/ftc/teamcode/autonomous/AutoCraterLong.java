package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto Crater Long", group ="Robot15173")
public class AutoCraterLong extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        runAutoMode();
    }

    @Override
    protected void act() {
        super.act();
        robot.encoderLift(0.5, 1,0, telemetry);
        move(0.8, 20);

        robot.encoderPivot(0.5, -5, 0, telemetry);

        move(0.8, 40);


        robot.encoderPivot(0.5, -4, 0, telemetry);

        move(0.8, 45);

        move(0.6, -100);


    }
}

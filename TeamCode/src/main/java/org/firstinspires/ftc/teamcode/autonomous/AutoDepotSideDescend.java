package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.gamefield.RobotLocation;
import org.firstinspires.ftc.teamcode.skills.GoldPosition;

@Autonomous(name="Depot", group ="Robot15173")
public class AutoDepotSideDescend extends AutoDepotSide {
    @Override
    public void runOpMode() throws InterruptedException {
        runAutoMode();
    }

    @Override
    protected void act() {
        shouldRaiseLift = false;
        descend();
        //lower lift
        robot.encoderLift(0.5, -2, 0, telemetry);

        checkPosition();

//        //find gold
//        GoldPosition gp = findGold(-1);
//        if (gp == GoldPosition.Left){
//            robot.encoderPivot(PIVOT_SPEED, -3, 0, telemetry);
//        }
//        else if (gp == GoldPosition.Center || gp == GoldPosition.None){
//
//        }else if (gp == GoldPosition.Right){
//            robot.encoderPivot(PIVOT_SPEED, 2, 0, telemetry);
//        }
//
//        move(DRIVE_SPEED, 24);

//        super.act();
    }
}

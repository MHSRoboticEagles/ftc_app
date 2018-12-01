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


        //lift arm
        //robot.encoderArm(1, 2, telemetry);
        //lower lift
        //robot.encoderLift(0.5, -2, 0, telemetry);

        //approach
        if (goldPosition == GoldPosition.Left){
            robot.encoderPivot(PIVOT_SPEED, -6, 0, telemetry);
            initArm();
            move(DRIVE_SPEED, 19);
            robot.encoderPivot(PIVOT_SPEED, 5, 0, telemetry);
            move(DRIVE_SPEED, 30);
            robot.dropMarker();
        }
        else if (goldPosition == GoldPosition.Center || goldPosition == GoldPosition.None){
            initArm();
            move(DRIVE_SPEED, 20);
            robot.encoderPivot(PIVOT_SPEED, -7, 0, telemetry);
            move(DRIVE_SPEED, 20);
            robot.dropMarker();

        }else if (goldPosition == GoldPosition.Right){
            robot.encoderPivot(PIVOT_SPEED, 6, 0, telemetry);
            initArm();

            move(DRIVE_SPEED, 24);
            robot.encoderPivot(PIVOT_SPEED, -7, 0, telemetry);
            move(DRIVE_SPEED, 18);
            robot.encoderPivot(PIVOT_SPEED, -12, 0, telemetry);
            move(DRIVE_SPEED, 20);
            robot.dropMarker();

        }


//        move(DRIVE_SPEED, 24);

//        super.act();
        while (opModeIsActive()){
            //do nothing
        }
    }
}

package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.RevDoubleBot;
import org.firstinspires.ftc.teamcode.gamefield.GameStats;
import org.firstinspires.ftc.teamcode.gamefield.RobotLocation;
import org.firstinspires.ftc.teamcode.skills.ColorCracker;
import org.firstinspires.ftc.teamcode.skills.CryptoColumn;
import org.firstinspires.ftc.teamcode.skills.DetectedColor;
import org.firstinspires.ftc.teamcode.skills.ImageRecognition;
import org.firstinspires.ftc.teamcode.skills.Navigator;


/**
 * Created by sjeltuhin on 1/15/18.
 */

public abstract class AutoBase extends LinearOpMode {
    protected boolean foundVuMark = false;

    protected RevDoubleBot robot = new RevDoubleBot();   // Use our standard robot configuration
    protected ElapsedTime runtime = new ElapsedTime();
    protected ImageRecognition imageRecognition = new ImageRecognition();
    protected static double TIME_CUT_OFF = 3.0;  //stop recognition at 5 sec. Then just guess.
    protected static float COLOR_CUT_OFF = 2;  //stop color detection at 3 sec.
    protected ColorCracker jewelHunter = new ColorCracker();
    protected DetectedColor dc = DetectedColor.NONE;
    protected static final double     DRIVE_SPEED             = 0.9;
    protected RobotLocation robotLocation = null;


    protected void runAutoMode(){
        robot.init(hardwareMap);

        initNav();

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

    }

    protected void initNav(){
        final Navigator nav = new Navigator(this);
        nav.setLocationChangedListener(new Navigator.LocationChangedListener() {
            @Override
            public boolean onNewLocation(RobotLocation loc) {
                boolean shouldStopTraking = false;
                robotLocation = loc;
                if (robotLocation.isActive()){
                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            robotLocation.X, robotLocation.Y, robotLocation.Z);
                    telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", robotLocation.rotateX, robotLocation.rotateY, robotLocation.rotateZ);
                }
                else{
                    telemetry.addData("Visible Target", "none");
                }
                telemetry.update();

                return shouldStopTraking;
            }
        });
        nav.init();
    }

    protected void move(double moveTo){
        telemetry.addData("Auto", "Distance = %.2f", moveTo);
        telemetry.update();
        robot.encoderDrive(DRIVE_SPEED, moveTo, moveTo, 0, telemetry);
        robot.stop();
    }

    protected void strafeInches(double moveTo){
        telemetry.addData("Auto", "Distance = %.2f", moveTo);
        telemetry.update();
        robot.encoderStrafe(DRIVE_SPEED, moveTo, 0, telemetry);
        robot.stop();
    }

    protected void turnRight(double degrees){
        robot.encoderDrive(DRIVE_SPEED/3, degrees, 0, 0, telemetry);
    }

    protected void turnLeft(double degrees){
        robot.encoderDrive(DRIVE_SPEED/3, -degrees, 0, 0, telemetry);
    }
}

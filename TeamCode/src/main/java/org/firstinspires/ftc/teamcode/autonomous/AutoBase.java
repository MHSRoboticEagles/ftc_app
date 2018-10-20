package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.RevDoubleBot;
import org.firstinspires.ftc.teamcode.gamefield.GameStats;
import org.firstinspires.ftc.teamcode.skills.ColorCracker;
import org.firstinspires.ftc.teamcode.skills.CryptoColumn;
import org.firstinspires.ftc.teamcode.skills.DetectedColor;
import org.firstinspires.ftc.teamcode.skills.ImageRecognition;


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


    protected void runAutoMode(){
        robot.init(hardwareMap);

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        crawl();

        turn();

        moveToCrater();


//        //descent to the floor
//        descent();
//
////        //strafe 2 inches left
////        strafeInches(2);
//
//
//        // turn 45 degrees right
//        turnRight(robot.TURN_45);
//
//        // go to the wall 32 inches
//        move(32);
//
//        // turn 90 degrees left
//        turnLeft(robot.TURN_90);
//
//        //move 24 inches into the crater
//        move(24);

    }

    protected void crawl(){

    }

    protected void turn(){

    }

    protected void moveToCrater(){

    }

    protected void descent(){

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
        robot.encoderDrive(DRIVE_SPEED, degrees, 0, 0, telemetry);
    }

    protected void turnLeft(double degrees){
        robot.encoderDrive(DRIVE_SPEED, -degrees, 0, 0, telemetry);
    }
}

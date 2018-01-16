package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.BasicBotConfig;
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
    protected static final double     DRIVE_SPEED             = 0.8;


    protected void runAutoMode(){
        robot.init(hardwareMap);
        jewelHunter.init(hardwareMap);

        imageRecognition.init(hardwareMap);

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        //do image recognition
        imageRecognition.start();
        runtime.reset();

        CryptoColumn result = CryptoColumn.None;

        boolean stop = false;

        while (opModeIsActive() && !stop) {
            stop = (runtime.seconds() >= TIME_CUT_OFF || foundVuMark);
            result = imageRecognition.track(telemetry);
            if (result != CryptoColumn.None){
                foundVuMark = true;
                break;
            }
        }
        if (opModeIsActive()) {
            proceed(result);
            sleep(1000);
            complete();
        }
    }

    protected void grabGlyph(){
        robot.sqeezeClaw();
        sleep(250);
        robot.moveLiftUp(telemetry);
    }

    protected void complete(){

    }

    protected void proceed(CryptoColumn column){
        jewel(column);
        position(column);
    }

    protected void jewel(CryptoColumn column){
        telemetry.addData("Auto", "Proceeding to column %s", column.name());
        kickJewel();
        grabGlyph();
        robot.initKickerTip();
    }

    protected void kickJewel(){
        telemetry.addData("Status", "Kicking the jewel");
        telemetry.update();
        //drop kicker
        robot.dropKicker();
        sleep(2000);
        try {
            runtime.reset();
            this.dc = jewelHunter.detectColor(telemetry, COLOR_CUT_OFF);
            sleep(1000);
            kick();
            telemetry.addData("Auto", "Color = %s", this.dc.name());
            telemetry.update();
        }
        catch (Exception ex){
            telemetry.addData("Issues with color sensor", ex);
            telemetry.update();
        }
        finally {
            sleep(1000);
            robot.liftKicker();
            sleep(1000);
        }
    }

    protected void kick(){

    }

    protected void redKick(){
        switch (this.dc){
            case RED:
                robot.kickEmptySide();
                break;
            case BLUE:
                robot.kickSensorSide();
                break;
            default:
                break;
        }
    }

    protected void blueKick(){
        switch (this.dc){
            case RED:
                robot.kickSensorSide();
                break;
            case BLUE:
                robot.kickEmptySide();
                break;
            default:
                break;
        }
    }


    protected void position(CryptoColumn column){
        switch (column){
            case Right:
                moveToRight();
                break;
            case Center:
                moveToCenter();
                break;
            case Left:
                moveToLeft();
                break;
            default:
                //let's go somewhere
                moveToCenter();
                break;
        }
    }

    protected void moveToRight(){
        telemetry.addData("Auto", "I am going to the right column");
        double moveTo = GameStats.DISTANCE_RIGHT;
        move(moveTo);
    }

    protected void moveToLeft(){
        telemetry.addData("VuMark", "I am going to the left cell");
        double moveTo = GameStats.DISTANCE_LEFT;
        move(moveTo);
    }

    protected void moveToCenter(){
        telemetry.addData("VuMark", "I am going to the center cell");
        double moveTo = GameStats.DISTANCE_CENTER;
        move(moveTo);
    }

    protected void move(double moveTo){
        telemetry.addData("Auto", "Distance = %.2f", moveTo);
        telemetry.update();
//        robot.moveToPosStraight(DRIVE_SPEED, 812, 812, 0, telemetry);
        robot.encoderDrive(DRIVE_SPEED, moveTo, moveTo, 0, telemetry);
        robot.stop();
    }

    protected void strafeToColumn(double moveTo){
        telemetry.addData("Auto", "Distance = %.2f", moveTo);
        telemetry.update();
        robot.encoderStrafe(DRIVE_SPEED, moveTo, 0, telemetry);
        robot.stop();
    }

    protected void completeTurn(){
        if (opModeIsActive()) {
            //back to right
            robot.encoderDrive(DRIVE_SPEED, -robot.TURN_45, 0, 0, telemetry);
            sleep(250);
            //forward to left
            robot.encoderDrive(DRIVE_SPEED, 0, robot.TURN_45, 0, telemetry);
            sleep(250);
            //approach
            robot.encoderDrive(DRIVE_SPEED, 5, 5, 0, telemetry);
            sleep(250);
            robot.openClaw();
            sleep(250);
            robot.encoderDrive(DRIVE_SPEED, -2, -2, 0, telemetry);
            robot.moveLiftDown(telemetry);
            telemetry.addData("Auto", "Done turning");
            telemetry.update();
        }
    }

    protected void completeStraight(){
        if (opModeIsActive()) {
            robot.encoderDrive(DRIVE_SPEED, 5, 5, 0, telemetry);
            sleep(250);
            robot.openClaw();
            sleep(250);
            robot.encoderDrive(DRIVE_SPEED, -2, -2, 0, telemetry);
            robot.moveLiftDown(telemetry);
            telemetry.addData("Auto", "Done turning");
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.bots.MixedDriveBot;
import org.firstinspires.ftc.teamcode.bots.RevDoubleBot;
import org.firstinspires.ftc.teamcode.gamefield.MineralLineUp;
import org.firstinspires.ftc.teamcode.gamefield.RobotLocation;
import org.firstinspires.ftc.teamcode.skills.ColorCracker;
import org.firstinspires.ftc.teamcode.skills.DetectedColor;
import org.firstinspires.ftc.teamcode.skills.GoldPosition;
import org.firstinspires.ftc.teamcode.skills.ImageRecognition;
import org.firstinspires.ftc.teamcode.skills.MineralDetection;
import org.firstinspires.ftc.teamcode.skills.Navigator;
import org.firstinspires.ftc.teamcode.skills.Parrot;
import org.firstinspires.ftc.teamcode.skills.VuforiaWrapper;


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
    protected static final double     DRIVE_SPEED             = 0.7;
    protected static final double     PIVOT_SPEED             = 0.7;
    protected RobotLocation robotLocation = null;
    protected Navigator nav;
    protected MineralDetection mineralDetection;
    protected GoldPosition goldPosition = GoldPosition.None;
    private VuforiaLocalizer vuforia = null;

    protected boolean shouldRaiseLift = true;


    protected void runAutoMode(){
        robot.init(hardwareMap);
        try {
            vuforia = VuforiaWrapper.initVuforia();

//            initNav();
            initGoldDetector();

            waitForStart();
            act();
        }
        catch (Exception ex){
            telemetry.addData("Issues autonomous initialization", ex);
            telemetry.update();
        }

    }


    protected void initNav(){
        nav = new Navigator(this.vuforia, this);
//        nav.setLocationChangedListener(new Navigator.LocationChangedListener() {
//            @Override
//            public boolean onNewLocation(RobotLocation loc) {
//                boolean shouldStopTracking = false;
//                robotLocation = loc;
//                shouldStopTraking = robotLocation.isActive();
//                if (robotLocation.isActive()){
//                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
//                            robotLocation.X, robotLocation.Y, robotLocation.Z);
//                    telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", robotLocation.rotateX, robotLocation.rotateY, robotLocation.rotateZ);
//                }
//                else{
//                    telemetry.addData("Visible Target", "none");
//                }
//                telemetry.update();
//
//                return shouldStopTracking;
//            }
//        });
        nav.init();
    }

    protected void initGoldDetector(){
        try {
            mineralDetection = new MineralDetection(this.vuforia, this.hardwareMap, telemetry);
            mineralDetection.setListener(new MineralDetection.MineralDetectionListener() {
                @Override
                public boolean onDetection(GoldPosition position) {
                    goldPosition = position;

                    if (position == GoldPosition.None) {
                        return false;
                    }

                    return true;
                }
            });
            mineralDetection.init();
        }
        catch (Exception ex){
            telemetry.addData("Gold", "Issues when initializing detector. %s", ex.getMessage());
            telemetry.update();
        }
    }

    protected RobotLocation checkPosition(){
        //check position
        robotLocation = nav.track(false, 1);
        if (robotLocation != null && robotLocation.isActive()){
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    robotLocation.X, robotLocation.Y, robotLocation.Z);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", robotLocation.rotateX, robotLocation.rotateY, robotLocation.rotateZ);
            telemetry.update();

        }
        return robotLocation;
    }

    protected GoldPosition findGold(int view, boolean withSound){
        MineralLineUp lineUp = mineralDetection.detectFlex(5, view);
        if (lineUp != null) {
            goldPosition = lineUp.getGoldPosition();
            if (withSound) {
            Parrot p = new Parrot(hardwareMap, telemetry);
            if (goldPosition != GoldPosition.None) {
                p.playSequence(goldPosition);
            }
            else{
                p.playClever();
            }
            }
            telemetry.addData("Line up: ", lineUp.toString());
            telemetry.update();
        }
        else{
            telemetry.addData("Minerals","Nothing detected. Not enough objects");
            telemetry.update();
        }
        mineralDetection.stopDetection();
        return goldPosition;
    }

    protected void descend (){
        robot.encoderLift(0.8, 7,0, telemetry);
//        strafeInches(3);
        robot.encoderPivot(PIVOT_SPEED, -4.2, 0, telemetry);
        move(DRIVE_SPEED, 1);
        robot.encoderPivot(PIVOT_SPEED, 1.5, 0, telemetry);
//        move(DRIVE_SPEED, 1);
    }

    protected void act(){

    }

    protected void runToTarget(){
        //approach
        if (goldPosition == GoldPosition.Left){
            robot.encoderPivot(PIVOT_SPEED, -6, 0, telemetry);
//            initArm();
            move(DRIVE_SPEED, 24);
            robot.encoderPivot(PIVOT_SPEED, 5, 0, telemetry);
            move(DRIVE_SPEED, 30);
            robot.dropMarker();
        }
        else if (goldPosition == GoldPosition.Center || goldPosition == GoldPosition.None){
//            initArm();
            move(DRIVE_SPEED, 21);
            robot.encoderPivot(PIVOT_SPEED, -7, 0, telemetry);
            move(DRIVE_SPEED, 30);
            robot.dropMarker();

        }else if (goldPosition == GoldPosition.Right){
            robot.encoderPivot(PIVOT_SPEED, 6, 0, telemetry);
//            initArm();

            move(DRIVE_SPEED, 24);
            robot.encoderPivot(PIVOT_SPEED, -7, 0, telemetry);
            move(DRIVE_SPEED, 18);
            robot.encoderPivot(PIVOT_SPEED, -12, 0, telemetry);
            move(DRIVE_SPEED, 25);
            robot.dropMarker();

        }
    }

    protected void runToTargetShort(){
        //approach
        if (goldPosition == GoldPosition.Left){
            robot.encoderPivot(PIVOT_SPEED, -6, 0, telemetry);
//            initArm();
            move(DRIVE_SPEED, 24);
            robot.encoderPivot(PIVOT_SPEED, 5, 0, telemetry);
            move(DRIVE_SPEED, 4);
        }
        else if (goldPosition == GoldPosition.Center || goldPosition == GoldPosition.None){
//            initArm();
            move(DRIVE_SPEED, 21);
            robot.encoderPivot(PIVOT_SPEED, -7, 0, telemetry);
            move(DRIVE_SPEED, 4);

        }else if (goldPosition == GoldPosition.Right){
            robot.encoderPivot(PIVOT_SPEED, 6, 0, telemetry);
//            initArm();

            move(DRIVE_SPEED, 24);
            robot.encoderPivot(PIVOT_SPEED, -7, 0, telemetry);
            move(DRIVE_SPEED, 4);
        }
    }

    protected void initArm(){
        robot.encoderArm(1, 6, telemetry);
        robot.encoderExtrude(0.2, 2, telemetry);
        robot.encoderExtrude(0.3, -1, telemetry);
    }


    protected void move(double speed, double moveTo){
        telemetry.addData("Auto", "Distance = %.2f", moveTo);
        telemetry.update();
        robot.encoderDrive(DRIVE_SPEED, moveTo, moveTo,0, telemetry);

        robot.stop();
    }

    protected void strafeInches(double moveTo){
        telemetry.addData("Auto", "Distance = %.2f", moveTo);
        telemetry.update();
        robot.encoderStrafe(DRIVE_SPEED, moveTo, 0, telemetry);
        robot.stop();
    }

    protected void turnRight(double degrees){
        if (robotLocation.isActive()){
            float initRotation = robotLocation.rotateZ;
            while (robotLocation.rotateX < initRotation + degrees){
                robot.pivotLeft(DRIVE_SPEED/3, telemetry);
                nav.track(true);
            }
        }
        else {
            robot.encoderDrive(DRIVE_SPEED / 3, degrees, 0, 0, telemetry);
        }
    }

    protected void turnLeft(double degrees){
        if (robotLocation.isActive()){
            float initRotation = robotLocation.rotateZ;
            while (robotLocation.rotateX > initRotation - degrees){
                robot.pivotLeft(DRIVE_SPEED/3, telemetry);
                nav.track(true);
            }
        }
        else {
            robot.encoderDrive(DRIVE_SPEED / 3, -degrees, 0, 0, telemetry);
        }
    }

}

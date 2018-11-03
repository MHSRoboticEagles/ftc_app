package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.RevDoubleBot;
import org.firstinspires.ftc.teamcode.gamefield.RobotLocation;
import org.firstinspires.ftc.teamcode.skills.ColorCracker;
import org.firstinspires.ftc.teamcode.skills.DetectedColor;
import org.firstinspires.ftc.teamcode.skills.GoldPosition;
import org.firstinspires.ftc.teamcode.skills.ImageRecognition;
import org.firstinspires.ftc.teamcode.skills.MineralDetection;
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
    protected static final double     DRIVE_SPEED             = 0.7;
    protected static final double     PIVOT_SPEED             = 0.7;
    protected RobotLocation robotLocation = null;
    protected Navigator nav;
    private MineralDetection mineralDetection;
    private GoldPosition goldPosition = GoldPosition.None;

    protected boolean shouldRaiseLift = true;


    protected void runAutoMode(){
        robot.init(hardwareMap);

//        initNav();
//        initGoldDetector();

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();
        act();
//        descend();
//        detectGold();

    }

    protected void initNav(){
        nav = new Navigator(this);
        nav.setLocationChangedListener(new Navigator.LocationChangedListener() {
            @Override
            public boolean onNewLocation(RobotLocation loc) {
                boolean shouldStopTraking = false;
                robotLocation = loc;
                shouldStopTraking = robotLocation.isActive();
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

    protected void initGoldDetector(){
        try {
            mineralDetection = new MineralDetection(this.hardwareMap, telemetry);
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

    protected void descend (){
        robot.encoderLift(0.8, 12.5,0, telemetry);
        strafeInches(3);
        robot.encoderPivot(PIVOT_SPEED, -3, 0, telemetry);
        move(DRIVE_SPEED, 2);
        robot.encoderPivot(PIVOT_SPEED, 3, 0, telemetry);
    }

    protected void act(){

    }

    protected void raiseArm(){
        robot.moveArmUp(0.4, telemetry);
        runtime.reset();
        while (runtime.seconds() <=2){

        }
        robot.stopArm();
    }

    protected void detectGold (){
        move(0.4, 10);
        mineralDetection.detectGold(3);
        telemetry.addData("Mineral", "Location. %s", goldPosition.name());
        telemetry.update();
    }

    protected void toDepot(){
        nav.track(1);
        if (robotLocation.isActive()){

        }
    }

    protected void toCrater(){

    }

    protected void move(double speed, double moveTo){
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
        if (robotLocation.isActive()){
            float initRotation = robotLocation.rotateZ;
            while (robotLocation.rotateX < initRotation + degrees){
                robot.pivotLeft(DRIVE_SPEED/3, telemetry);
                nav.track();
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
                nav.track();
            }
        }
        else {
            robot.encoderDrive(DRIVE_SPEED / 3, -degrees, 0, 0, telemetry);
        }
    }

}

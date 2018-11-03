package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.skills.ColorCracker;
import org.firstinspires.ftc.teamcode.skills.ColorDistance;
import org.firstinspires.ftc.teamcode.skills.DetectedColor;

import static java.lang.Thread.sleep;

/**
 * Created by sjeltuhin on 9/12/17.
 */

public class RevDoubleBot {
    public DcMotor leftDriveBack = null;
    public DcMotor rightDriveBack = null;

    public DcMotor leftDriveFront = null;
    public DcMotor rightDriveFront = null;

//    public DcMotor scoop = null;
    public DcMotor arm = null;

    public boolean armStopped = false;


    public DcMotor lift = null;
    private double liftPos = 0;

    private CRServo scoop = null;

    private Servo elbow = null;

//    private ColorDistance colorCracker = null;


    private ElapsedTime     runtime = new ElapsedTime();



    private static double [] liftStepDegrees = {95, 130};
    private int currentStep = 0;
    private static final int LIFT_MAX_STEPS = 2;

    private double armBasePos = 0;
    private static final double ARM_RANGE = 180;

    private double elbowPos = 0;
    private static final double ELBOW_RANGE = 300;

    private double ANTI_GRAVITY_POWER = 0.01;

    private double LIFT_SPEED = 0.5;
    private double LIFT_SPEED_DOWN = 0.2;
    public double DRIVE_SPEED = 0.9;




    //REV

    static final double     COUNTS_PER_MOTOR_REV    = 288 ;    // Rev Core Hex motor
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP. was 2 in the sample
    static final double     WHEEL_DIAMETER_INCHES   = 4.05 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH_REV     = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);  //22.64
    static final double     COUNTS_PER_DEGREE_REV    = COUNTS_PER_MOTOR_REV/360;


    //Rev HD
    static final double     COUNTS_PERMOTOR_REV_HD   = 1120;   // Rev HD motor
    static final double     COUNTS_PER_INCH_REV_HD     = (COUNTS_PERMOTOR_REV_HD * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);


    static final double     STRAFE_INCH    = 1.5 ;    // Rev Core Hex motor

    //AndyMark
    static final double     COUNTS_PER_MOTOR_AM    = 718 ;    // eg: AndyMark Motor Encoder
    static final double     COUNTS_PER_DEGREE_AM    = COUNTS_PER_MOTOR_AM/360 ;

    static final double     COUNTS_PER_MOTOR_TQ    = 1440 ;    // eg: Torquenado Motor Encoder
    static final double     COUNTS_PER_DEGREE_TQ    = COUNTS_PER_MOTOR_TQ/360 ;



    //lift
    static final double     COUNTS_PER_INCH_LIFT_REV     = (COUNTS_PERMOTOR_REV_HD * DRIVE_GEAR_REDUCTION) /
            (1 * 3.1415);

    static final double     MAX_DISTANCE_INCHES  = 7.0 ; //inches
    static final double     MAX_POSITION_POSITION         = MAX_DISTANCE_INCHES * COUNTS_PER_INCH_LIFT_REV;

    public static final double TURN_45                 = 22.5; //45% turn

    public static final double TURN_90                = TURN_45*2;


    //callbacks
    public interface RotationChangedListener{
        void onArmMoved(double degreeChanged, double currentAngle);
    }

    private RotationChangedListener rotationChangedListener = null;

    public RotationChangedListener getRotationChangedListener() {
        return rotationChangedListener;
    }

    public void setRotationChangedListener(RotationChangedListener rotationChangedListener) {
        this.rotationChangedListener = rotationChangedListener;
    }


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public RevDoubleBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDriveBack  = hwMap.get(DcMotor.class, "left_drive_back");
        rightDriveBack = hwMap.get(DcMotor.class, "right_drive_back");
        leftDriveFront  = hwMap.get(DcMotor.class, "left_drive_front");
        rightDriveFront = hwMap.get(DcMotor.class, "right_drive_front");

        leftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);
        leftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveFront.setDirection(DcMotor.Direction.FORWARD);

        resetEncoders();

        leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set all motors to zero power
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);
        leftDriveFront.setPower(0);
        rightDriveFront.setPower(0);


        //lift
        lift = hwMap.get(DcMotor.class, "lift");
        lift.setDirection(DcMotor.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




//        //color sensor
//        colorCracker = new ColorDistance();
//        colorCracker.init(ahwMap);

        this.liftStop();


        scoop = hwMap.get(CRServo.class, "scoop");
        scoop.setPower(0);
        scoop.setDirection(CRServo.Direction.REVERSE);


        arm = hwMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setPower(0);

        elbow = hwMap.get(Servo.class, "elbow");
        elbow.setPosition(0);
    }

    protected void resetEncoders(){
        leftDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void initMode(DcMotor.RunMode mode){
        leftDriveBack.setMode(mode);
        rightDriveBack.setMode(mode);
        leftDriveFront.setMode(mode);
        rightDriveFront.setMode(mode);
    }

    public void stop (){
        this.leftDriveBack.setPower(0);
        this.rightDriveBack.setPower(0);
        this.leftDriveFront.setPower(0);
        this.rightDriveFront.setPower(0);
    }

    public void move(double drive, double turn, Telemetry telemetry){
        double rightPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        double leftPower   = Range.clip(drive - turn, -1.0, 1.0) ;
        //use cubic modifier
        rightPower = rightPower*rightPower*rightPower;
        leftPower = leftPower*leftPower*leftPower;

        this.leftDriveBack.setPower(leftPower);
        this.rightDriveBack.setPower(rightPower);
        this.leftDriveFront.setPower(leftPower);
        this.rightDriveFront.setPower(rightPower);
        telemetry.addData("Motors", "Power: %.0f", drive);
//        telemetry.update();
    }

    public void moveArm(double val, Telemetry telemetry){
        double power = Range.clip(val, -1.0, 1.0) ;

        //use cubic modifier
        power = power*power*power;

        if (val < 0){
            power = power / 2;
        }

        this.arm.setPower(power);

        telemetry.addData("Motors", "Power: %.0f", val);
    }

    public void moveArmUp(double val, Telemetry telemetry){
        double power = Range.clip(val, -1.0, 1.0) ;

        //use cubic modifier
        power = power*power*power / 2;

        this.arm.setPower(-power);

        telemetry.addData("Motors", "Power: %.0f", val);
    }

    public void extendElbow(){
//        double p = Range.clip(val, 0, 0.15) ;
        this.elbow.setPosition(0.3);
    }

    public void collapselbow(){
//        double p = Range.clip(val, -0.15, 0) ;
        this.elbow.setPosition(0);
    }

    public void rotateScoop(double val, Telemetry telemetry){
        double power = Range.clip(val, -1.0, 1.0) ;

//        //use cubic modifier
//        power = power*power*power;

        this.scoop.setPower(power);

//        telemetry.addData("Motors", "Power: %.0f", val);
    }

    public void strafeLeft(double speed){
        double power    = Range.clip(speed, -1.0, 1.0) ;
        this.leftDriveBack.setPower(power);
        this.rightDriveBack.setPower(-power);
        this.leftDriveFront.setPower(-power);
        this.rightDriveFront.setPower(power);
    }

    public void strafeRight(double speed){
        double power    = Range.clip(speed, -1.0, 1.0) ;
        this.leftDriveBack.setPower(-power);
        this.rightDriveBack.setPower(power);
        this.leftDriveFront.setPower(power);
        this.rightDriveFront.setPower(-power);
    }

    public void pivotLeft(double speed, Telemetry telemetry){
        this.leftDriveBack.setPower(-speed);
        this.rightDriveBack.setPower(speed);
        this.leftDriveFront.setPower(-speed);
        this.rightDriveFront.setPower(speed);
        telemetry.addData("Motors", "Left: %7d Right: %7d", leftDriveFront.getCurrentPosition(), rightDriveFront.getCurrentPosition());
        telemetry.update();
    }

    public void pivotRight(double speed, Telemetry telemetry){
        this.leftDriveBack.setPower(speed);
        this.rightDriveBack.setPower(-speed);
        this.leftDriveFront.setPower(speed);
        this.rightDriveFront.setPower(-speed);
        telemetry.addData("Motors", "Lef: %7d Right: %7d", leftDriveFront.getCurrentPosition(), rightDriveFront.getCurrentPosition());
        telemetry.update();
    }

    public void turnLeft(double speed){
        this.leftDriveBack.setPower(0);
        this.rightDriveBack.setPower(speed);
        this.leftDriveFront.setPower(0);
        this.rightDriveFront.setPower(speed);
    }

    public void turnRight(double speed){
        this.leftDriveBack.setPower(speed);
        this.rightDriveBack.setPower(0);
        this.leftDriveFront.setPower(speed);
        this.rightDriveFront.setPower(0);
    }

    public void moveLift(double speed, Telemetry telemetry){
        if (speed < 0){
            moveLiftDown(speed, telemetry);
        }
        else{
            moveLiftUp(speed, telemetry);
        }
    }

    public void moveLiftUp(double speed, Telemetry telemetry){
        this.lift.setPower(speed);
        liftPos = this.lift.getCurrentPosition();
        telemetry.addData("Lift", "Position: %.2f", this.liftPos);
        telemetry.update();
    }

    public void moveLiftDown(double speed, Telemetry telemetry){

        this.lift.setPower(speed);
        liftPos = this.lift.getCurrentPosition();

        telemetry.addData("Lift", "Position: %.2f", this.liftPos);
        telemetry.update();
    }


    public void liftStop(){
        this.lift.setPower(0);
    }



    private void moveMotorDegrees(DcMotor motor, double speed, double degrees, double motorCounts, Telemetry telemetry){
        try{
            int newTarget = motor.getCurrentPosition() + (int) (degrees * motorCounts);
            motor.setTargetPosition(newTarget);

            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            motor.setPower(Math.abs(speed));

            boolean stop = false;
            while (!stop) {
                stop = !motor.isBusy();
                // Display it for the driver.
                telemetry.addData("M", "Running to %7d", newTarget);
                telemetry.addData("M", "Current: %7d",
                        motor.getCurrentPosition());
                telemetry.update();
            }

            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower(ANTI_GRAVITY_POWER);
        }
        catch (Exception ex){
            telemetry.addData("Issues running with encoders to position", ex);
            telemetry.update();
        }
    }


    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS, Telemetry telemetry) {

        try {
            // Determine new target position, and pass to motor controller
            int newLeftTarget = this.leftDriveBack.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH_REV);
            int newRightTarget = this.rightDriveBack.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH_REV);
            int newLeftFrontTarget = this.leftDriveFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH_REV);
            int newRightFrontTarget = this.rightDriveFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH_REV);

            this.leftDriveBack.setTargetPosition(newLeftTarget);
            this.rightDriveBack.setTargetPosition(newRightTarget);
            this.leftDriveFront.setTargetPosition(newLeftFrontTarget);
            this.rightDriveFront.setTargetPosition(newRightFrontTarget);


            // Turn On RUN_TO_POSITION
            leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            this.leftDriveBack.setPower(Math.abs(speed));
            this.rightDriveBack.setPower(Math.abs(speed));
            this.leftDriveFront.setPower(Math.abs(speed));
            this.rightDriveFront.setPower(Math.abs(speed));

            boolean stop = false;
            boolean leftMove = leftInches != 0;
            boolean rightMove = rightInches != 0;
            while (!stop) {
                boolean timeUp = timeoutS > 0 && runtime.seconds() >= timeoutS;
                stop = timeUp || ((leftMove && !this.leftDriveBack.isBusy()) || (rightMove && !this.rightDriveBack.isBusy())
                || (leftMove && !this.leftDriveFront.isBusy()) || (rightMove && !this.rightDriveFront.isBusy()));

                telemetry.addData("Motors", "Starting encoder drive. Left: %.2f, Right:%.2f", leftInches, rightInches);
                telemetry.addData("Motors", "LeftFront from %7d to %7d", leftDriveFront.getCurrentPosition(), newLeftFrontTarget);
                telemetry.addData("Motors", "LeftBack from %7d to %7d", leftDriveBack.getCurrentPosition(), newLeftTarget);
                telemetry.addData("Motors", "RightFront from %7d to %7d", rightDriveFront.getCurrentPosition(), newRightFrontTarget);
                telemetry.addData("Motors", "RightBack from %7d to %7d", rightDriveBack.getCurrentPosition(), newRightTarget);
                telemetry.update();
            }

            telemetry.addData("Motors", "Going to stop");
            telemetry.update();
            this.stop();
            telemetry.addData("Motors", "Stopped");
            telemetry.update();

            // Turn off RUN_TO_POSITION
            leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception ex){
            telemetry.addData("Issues running with encoders to position", ex);
            telemetry.update();
        }
    }

    public void encoderStrafe(double speed,
                             double distanceInches,
                             double timeoutS, Telemetry telemetry) {

        try {
            double val = Math.abs(distanceInches);
            int newLeftTarget = 0 ;
            int newRightTarget = 0;
            int newLeftFrontTarget = 0;
            int newRightFrontTarget = 0;
            int increment = (int) (val * COUNTS_PER_INCH_REV * STRAFE_INCH);;
            if (distanceInches < 0){
                //going left
                newLeftTarget = this.leftDriveBack.getCurrentPosition() + increment;
                newRightTarget = this.rightDriveBack.getCurrentPosition() - increment;
                newLeftFrontTarget = this.leftDriveFront.getCurrentPosition() - increment;
                newRightFrontTarget = this.rightDriveFront.getCurrentPosition() + increment;
            }
            else{
                //going right
                newLeftTarget = this.leftDriveBack.getCurrentPosition() - increment;
                newRightTarget = this.rightDriveBack.getCurrentPosition() + increment;
                newLeftFrontTarget = this.leftDriveFront.getCurrentPosition() + increment;
                newRightFrontTarget = this.rightDriveFront.getCurrentPosition() - increment;
            }


            this.leftDriveBack.setTargetPosition(newLeftTarget);
            this.rightDriveBack.setTargetPosition(newRightTarget);
            this.leftDriveFront.setTargetPosition(newLeftFrontTarget);
            this.rightDriveFront.setTargetPosition(newRightFrontTarget);


            // Turn On RUN_TO_POSITION
            leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            this.leftDriveBack.setPower(Math.abs(speed));
            this.rightDriveBack.setPower(Math.abs(speed));
            this.leftDriveFront.setPower(Math.abs(speed));
            this.rightDriveFront.setPower(Math.abs(speed));

            boolean stop = false;
            while (!stop) {
                boolean timeUp = timeoutS > 0 && runtime.seconds() >= timeoutS;
                stop = timeUp || (!this.leftDriveBack.isBusy() || !this.rightDriveBack.isBusy()
                        || !this.leftDriveFront.isBusy() || !this.rightDriveFront.isBusy());
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Back: %7d :%7d front: %7d :%7d",
                        this.leftDriveBack.getCurrentPosition(),
                        this.rightDriveBack.getCurrentPosition(),
                        this.leftDriveFront.getCurrentPosition(),
                        this.rightDriveFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            this.stop();

            // Turn off RUN_TO_POSITION
            leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception ex){
            telemetry.addData("Issues running with encoders to position", ex);
            telemetry.update();
        }
    }

    public void encoderPivot(double speed,
                              double inches,
                              double timeoutS, Telemetry telemetry) {

        try {
            double val = Math.abs(inches);
            int newLeftTarget = 0 ;
            int newRightTarget = 0;
            int newLeftFrontTarget = 0;
            int newRightFrontTarget = 0;
            if (inches < 0){
                //pivot left
                newLeftTarget = this.leftDriveBack.getCurrentPosition() - (int) (val * COUNTS_PER_INCH_REV_HD);
                newRightTarget = this.rightDriveBack.getCurrentPosition() + (int) (val * COUNTS_PER_INCH_REV_HD);
                newLeftFrontTarget = this.leftDriveFront.getCurrentPosition() - (int) (val * COUNTS_PER_INCH_REV_HD);
                newRightFrontTarget = this.rightDriveFront.getCurrentPosition() + (int) (val * COUNTS_PER_INCH_REV_HD);
            }
            else{
                //pivot right
                newLeftTarget = this.leftDriveBack.getCurrentPosition() + (int) (val * COUNTS_PER_INCH_REV_HD);
                newRightTarget = this.rightDriveBack.getCurrentPosition() - (int) (val * COUNTS_PER_INCH_REV_HD);
                newLeftFrontTarget = this.leftDriveFront.getCurrentPosition() + (int) (val * COUNTS_PER_INCH_REV_HD);
                newRightFrontTarget = this.rightDriveFront.getCurrentPosition() - (int) (val * COUNTS_PER_INCH_REV_HD);
            }


            this.leftDriveBack.setTargetPosition(newLeftTarget);
            this.rightDriveBack.setTargetPosition(newRightTarget);
            this.leftDriveFront.setTargetPosition(newLeftFrontTarget);
            this.rightDriveFront.setTargetPosition(newRightFrontTarget);


            // Turn On RUN_TO_POSITION
            leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            this.leftDriveBack.setPower(Math.abs(speed));
            this.rightDriveBack.setPower(Math.abs(speed));
            this.leftDriveFront.setPower(Math.abs(speed));
            this.rightDriveFront.setPower(Math.abs(speed));

            boolean stop = false;
            while (!stop) {
                boolean timeUp = timeoutS > 0 && runtime.seconds() >= timeoutS;
                stop = timeUp || (!this.leftDriveBack.isBusy() || !this.rightDriveBack.isBusy()
                        || !this.leftDriveFront.isBusy() || !this.rightDriveFront.isBusy());
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Back: %7d :%7d front: %7d :%7d",
                        this.leftDriveBack.getCurrentPosition(),
                        this.rightDriveBack.getCurrentPosition(),
                        this.leftDriveFront.getCurrentPosition(),
                        this.rightDriveFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            this.stop();

            // Turn off RUN_TO_POSITION
            leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception ex){
            telemetry.addData("Issues running with encoders to position", ex);
            telemetry.update();
        }
    }

//    public void moveClawSimple(double drive, Telemetry telemetry){
//        double rightPower    = Range.clip(drive, -1.0, 1.0) ;
//        double leftPower   = Range.clip(drive, -1.0, 1.0) ;
//
//        this.leftArmBase.setPower(leftPower);
//        this.rightArmBase.setPower(rightPower);
//
//        telemetry.addData("Arm", "Left Pos to %7d", leftArmBase.getCurrentPosition());
//        telemetry.update();
//
//    }

    public void encoderLift(double speed,
                             double inches, double timeoutS, Telemetry telemetry) {

        try {

            if (inches < 0){
                speed = -speed;
            }

            int newTarget = this.lift.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_LIFT_REV);


            this.lift.setTargetPosition(newTarget);


            // Turn On RUN_TO_POSITION
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            this.lift.setPower(Math.abs(speed));

            boolean stop = false;
            while (!stop) {
                boolean timeUp = timeoutS > 0 && runtime.seconds() >= timeoutS;
                stop = timeUp || !this.lift.isBusy();
                // Display it for the driver.
                telemetry.addData("Lift", "Running to %7d", newTarget);
                telemetry.addData("Lift", "Current Pos: %7d", this.lift.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;

            this.lift.setPower(0);

            // Turn off RUN_TO_POSITION
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception ex){
            telemetry.addData("Issues running with encoders to position", ex);
            telemetry.update();
        }
    }

    public void toggleArm(){
        if (this.armStopped == true){
            this.armStopped = false;
        }
        else{
            this.armStopped = true;
        }
        if (this.armStopped){
            this.arm.setPower(ANTI_GRAVITY_POWER);
        }
        else{
            this.arm.setPower(0);
        }
    }

    public  boolean isArmStopped(){
        return this.armStopped;
    }

//    public DetectedColor checkColor(Telemetry telemetry, float timeout){
//        return colorCracker.detectColor(telemetry, timeout);
//    }
}

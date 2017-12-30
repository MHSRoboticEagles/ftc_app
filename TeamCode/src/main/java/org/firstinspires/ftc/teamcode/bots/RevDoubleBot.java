package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by sjeltuhin on 9/12/17.
 */

public class RevDoubleBot {
    public DcMotor leftDriveBack = null;
    public DcMotor rightDriveBack = null;

    public DcMotor leftDriveFront = null;
    public DcMotor rightDriveFront = null;

    public DcMotor leftArmBase = null;
    public DcMotor rightArmBase = null;
    public DcMotor elbow = null;


    public Servo lift = null;
    private double liftPos = 0;

    public Servo    leftClaw    = null;
    public Servo    rightClaw   = null;
    public Servo    jewelKicker   = null;
    public Servo    relicClaw    = null;

    private boolean clawShut = false;

    private ElapsedTime     runtime = new ElapsedTime();

    private static final double SERVO_START_VALUE = 0.42;
    private static final double KICKER_UP_VALUE = 1;
    private static final double KICKER_DOWN_VALUE = 0.19;
    private static final double LEFT_CLAW_START = SERVO_START_VALUE;
    private static final double RIGHT_CLAW_START = 1 - SERVO_START_VALUE;
    public static final double LIFT_INCREMENT = 0.15;
    private int currentStep = 0;
    private static final int LIFT_MAX_STEPS = 4;

    private static final double RELIC_CLAW_OPEN = 0;
    private static final double RELIC_CLAW_SHUT = 8;

    static final double     COUNTS_PER_MOTOR_REV    = 288 ;    // Rev Core Hex motor
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP. was 2 in the sample
    static final double     WHEEL_DIAMETER_INCHES   = 4.05 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     COUNTS_PER_DEGREE    = COUNTS_PER_MOTOR_REV/360 ;


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
        rightDriveBack.setDirection(DcMotor.Direction.REVERSE);
        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
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
        lift = hwMap.get(Servo.class, "lift");
        this.liftStop(liftPos);

        //claws
        leftClaw  = hwMap.get(Servo.class, "left_claw");
        rightClaw = hwMap.get(Servo.class, "right_claw");
        this.leftClaw.scaleRange(0, 1);
        this.rightClaw.scaleRange(0, 1);

        this.leftClaw.setPosition(LEFT_CLAW_START);
        this.rightClaw.setPosition(RIGHT_CLAW_START);

        //jewelkicker
        jewelKicker = hwMap.get(Servo.class, "kicker");
        this.jewelKicker.scaleRange(0, 1);
        this.jewelKicker.setPosition(KICKER_UP_VALUE);


        //relic arm
        leftArmBase = hwMap.get(DcMotor.class, "left_arm");
        rightArmBase = hwMap.get(DcMotor.class, "right_arm");
        elbow = hwMap.get(DcMotor.class, "elbow");

        leftArmBase.setDirection(DcMotor.Direction.FORWARD);
        rightArmBase.setDirection(DcMotor.Direction.REVERSE);
        elbow.setDirection(DcMotor.Direction.FORWARD);

        leftArmBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftArmBase.setPower(0);
        rightArmBase.setPower(0);
        elbow.setPower(0);

        relicClaw = hwMap.get(Servo.class, "kicker");
        relicClaw.scaleRange(0, 1);
        relicClaw.setPosition(RELIC_CLAW_OPEN);
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

    public void move(double drive, double turn){
        double rightPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        double leftPower   = Range.clip(drive - turn, -1.0, 1.0) ;
        //use cubic modifier
        rightPower = rightPower*rightPower*rightPower;
        leftPower = leftPower*leftPower*leftPower;

        this.leftDriveBack.setPower(leftPower);
        this.rightDriveBack.setPower(rightPower);
        this.leftDriveFront.setPower(leftPower);
        this.rightDriveFront.setPower(rightPower);
    }


    public void strifeLeft(double speed){
        double power    = Range.clip(speed, -1.0, 1.0) ;
        this.leftDriveBack.setPower(power);
        this.rightDriveBack.setPower(-power);
        this.leftDriveFront.setPower(-power);
        this.rightDriveFront.setPower(power);
    }

    public void strifeRight(double speed){
        double power    = Range.clip(speed, -1.0, 1.0) ;
        this.leftDriveBack.setPower(-power);
        this.rightDriveBack.setPower(power);
        this.leftDriveFront.setPower(power);
        this.rightDriveFront.setPower(-power);
    }

    public void strifeLeftPos(double speed, double posInches){
        double power    = Range.clip(speed, -1.0, 1.0) ;
        this.leftDriveBack.setPower(power);
        this.rightDriveBack.setPower(-power);
        this.leftDriveFront.setPower(-power);
        this.rightDriveFront.setPower(power);
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

    public void rotateLeftClaw(double position){
        double p = Range.clip(LEFT_CLAW_START + position, LEFT_CLAW_START, 1.0);
        this.leftClaw.setPosition(p);
    }

    public void rotateRightClaw(double position){
        double p = Range.clip(RIGHT_CLAW_START - position, 0, RIGHT_CLAW_START);
        this.rightClaw.setPosition(p);
    }

    public void moveClaw(double position){
        this.rotateLeftClaw(position);
        this.rotateRightClaw(position);
    }

    public void sqeezeClaw(){
        this.leftClaw.setPosition(0.95);
        this.rightClaw.setPosition(0.05);
    }

    public void openClaw(){
        this.leftClaw.setPosition(LEFT_CLAW_START);
        this.rightClaw.setPosition(RIGHT_CLAW_START);
    }

    public double liftUp(){
        if (currentStep >= LIFT_MAX_STEPS){
            return liftPos;
        }
        currentStep++;
        double liftPower  = Range.clip(LIFT_INCREMENT* currentStep, 0, 1);
        return moveLift(liftPower);
    }

    public double liftDown(){
        if (currentStep <= 0){
            return liftPos;
        }
        currentStep--;
        double liftPower  = Range.clip(LIFT_INCREMENT* currentStep, 0, 1);
        return moveLift(liftPower);
    }

    public void liftStop(double pos){
        this.lift.setPosition(pos);
    }

    protected double moveLift(double pos){
        this.lift.setPosition(pos);
        this.liftPos = pos;
        return liftPos;
    }

    public void openRelicClaw(){
        this.relicClaw.setPosition(RELIC_CLAW_OPEN);
        this.clawShut = false;
    }

    public void closeRelicClaw(){
        this.relicClaw.setPosition(RELIC_CLAW_SHUT);
        this.clawShut = true;
    }

    public boolean isRelicClawShut(){
        return this.clawShut;
    }

    public void moveArm(double speed, double val, Telemetry telemetry){
        try{
            double degrees = val*180;
            int newLeftTarget = this.leftArmBase.getCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
            int newRightTarget = this.rightArmBase.getCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
            this.leftArmBase.setTargetPosition(newLeftTarget);
            this.rightArmBase.setTargetPosition(newRightTarget);

            runtime.reset();
            this.leftArmBase.setPower(Math.abs(speed));
            this.rightArmBase.setPower(Math.abs(speed));

            boolean stop = false;
            while (!stop) {
                stop =  (!this.leftArmBase.isBusy() || !this.rightArmBase.isBusy());
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Arms: %7d :%7d",
                        this.leftArmBase.getCurrentPosition(),
                        this.rightArmBase.getCurrentPosition());
                telemetry.update();
            }
        }
        catch (Exception ex){
            telemetry.addData("Issues running with encoders to position", ex);
            telemetry.update();
        }
    }

    public void moveElbow(double speed, double val, Telemetry telemetry){
        try{
            double degrees = val*180;
            int newTarget = this.elbow.getCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
            this.elbow.setTargetPosition(newTarget);

            runtime.reset();
            this.elbow.setPower(Math.abs(speed));

            boolean stop = false;
            while (!stop) {
                stop = !this.elbow.isBusy();
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d", newTarget);
                telemetry.addData("Path2", "Elbow: %7d",
                        this.elbow.getCurrentPosition());
                telemetry.update();
            }
        }
        catch (Exception ex){
            telemetry.addData("Issues running with encoders to position", ex);
            telemetry.update();
        }
    }

    public double getLiftPos(){
        return this.lift.getPosition();
    }

    public void dropKicker(){
        jewelKicker.setPosition(KICKER_DOWN_VALUE);
    }

    public void liftKicker(){
        jewelKicker.setPosition(KICKER_UP_VALUE);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS, Telemetry telemetry) {

        try {
            // Determine new target position, and pass to motor controller
            int newLeftTarget = this.leftDriveBack.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            int newRightTarget = this.rightDriveBack.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            int newLeftFrontTarget = this.leftDriveFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            int newRightFrontTarget = this.rightDriveFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            this.leftDriveBack.setTargetPosition(newLeftTarget);
            this.rightDriveBack.setTargetPosition(newRightTarget);
            this.leftDriveFront.setTargetPosition(newLeftFrontTarget);
            this.rightDriveFront.setTargetPosition(newRightFrontTarget);


            // Turn On RUN_TO_POSITION
            leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
                || this.leftDriveFront.isBusy() || this.rightDriveFront.isBusy());
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
}

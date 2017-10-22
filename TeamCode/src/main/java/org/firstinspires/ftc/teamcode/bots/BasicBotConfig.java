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

public class BasicBotConfig {
    public DcMotor leftDriveBack = null;
    public DcMotor rightDriveBack = null;

    public DcMotor leftDriveFront = null;
    public DcMotor rightDriveFront = null;

    public DcMotor arm = null;

    public Servo    leftClaw    = null;
    public Servo    rightClaw   = null;
    public Servo    jewelKicker   = null;
    public Servo    wrist   = null;
    private ElapsedTime     runtime = new ElapsedTime();

    private static final double WRIST_DEFAULT_VALUE = 0.2;
    private static final double SERVO_START_VALUE = 0;
    private static final double KICKER_UP_VALUE = 0;
    private static final double KICKER_DOWN_VALUE = 0.65;
    private static final double LEFT_CLAW_START = SERVO_START_VALUE;
    private static final double RIGHT_CLAW_START = 1 - SERVO_START_VALUE;
    public static final double LENGTH = 18;

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: AndyMark Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP. was 2 in the sample
    static final double     WHEEL_DIAMETER_INCHES   = 3.85 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public BasicBotConfig(){

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

        initMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set all motors to zero power
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);
        leftDriveFront.setPower(0);
        rightDriveFront.setPower(0);


        //arm
        arm = hwMap.get(DcMotor.class, "arm");
        //wrist  = hwMap.get(Servo.class, "wrist");

        //wrist.setDirection(Servo.Direction.REVERSE);
        //wrist.scaleRange(0, 1);
        //wrist.setPosition(WRIST_DEFAULT_VALUE);

        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setPower(0);

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
        //left side is dragging. increase power by 25%
        leftPower = Range.clip(leftPower + leftPower*0.45, -1.0, 1.0) ;
        this.leftDriveBack.setPower(leftPower);
        this.rightDriveBack.setPower(rightPower);
        this.leftDriveFront.setPower(leftPower);
        this.rightDriveFront.setPower(rightPower);
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

//    public void moveWrist(double position){
//        this.wrist.setPosition(position);
//    }

    public int liftArm(double speed, int seconds, Telemetry telemetry){
        return moveArm(speed, seconds, telemetry);
    }

    public int lowerArm(double speed, int seconds, Telemetry telemetry){
        return moveArm(-speed, seconds, telemetry);
    }

    public int moveArm(double speed, int seconds, Telemetry telemetry){
        runtime.reset();
        double armPower  = Range.clip(speed, -1.0, 1.0)/2 ;
        this.arm.setPower(armPower);
        telemetry.addData("Arm", "Current Position = %7d", this.arm.getCurrentPosition());
        telemetry.update();
        if (seconds > 0) {
            while (runtime.seconds() < seconds) {
                //operate arm for the given number of seconds
            }
            this.arm.setPower(0);
        }
        return this.arm.getCurrentPosition();
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
            int newLeftTargetFront = this.leftDriveFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            int newRightTargetFront = this.rightDriveFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            this.leftDriveBack.setTargetPosition(newLeftTarget);
            this.rightDriveBack.setTargetPosition(newRightTarget);
            this.leftDriveFront.setTargetPosition(newLeftTargetFront);
            this.rightDriveFront.setTargetPosition(newRightTargetFront);


            // Turn On RUN_TO_POSITION
            this.initMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            this.leftDriveBack.setPower(Math.abs(speed));
            this.rightDriveBack.setPower(Math.abs(speed));
            this.leftDriveFront.setPower(Math.abs(speed));
            this.rightDriveFront.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the this will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the this continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            boolean stop = false;
            while (!stop) {
                boolean timeUp = timeoutS > 0 && runtime.seconds() >= timeoutS;
                stop = timeUp || (!this.leftDriveBack.isBusy() || !this.rightDriveBack.isBusy() ||
                !this.rightDriveFront.isBusy() || !this.leftDriveFront.isBusy());
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        this.leftDriveBack.getCurrentPosition(),
                        this.rightDriveBack.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            this.stop();

            // Turn off RUN_TO_POSITION
            this.initMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception ex){
            telemetry.addData("Issues running with encoders to position", ex);
            telemetry.update();
        }
    }

}

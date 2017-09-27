package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by sjeltuhin on 9/12/17.
 */

public class BasicBotConfig {
    public DcMotor leftDriveBack = null;
    public DcMotor rightDriveBack = null;

    public Servo    leftClaw    = null;
    public Servo    rightClaw   = null;

    private static final double SERVO_START_VALUE = 0.4;
    private static final double LEFT_CLAW_START = SERVO_START_VALUE;
    private static final double RIGHT_CLAW_START = 1 - SERVO_START_VALUE;


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
        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        rightDriveBack.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);

        leftDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Set all motors to run with encoders.
        leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //servos
        leftClaw  = hwMap.get(Servo.class, "left_claw");
        rightClaw = hwMap.get(Servo.class, "right_claw");

        this.leftClaw.setPosition(LEFT_CLAW_START);
        this.rightClaw.setPosition(1 - RIGHT_CLAW_START);
    }

    public void stop (){
        this.leftDriveBack.setPower(0);
        this.rightDriveBack.setPower(0);
    }

    public void move(double drive, double turn){
        double leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        double rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
        this.leftDriveBack.setPower(leftPower);
        this.rightDriveBack.setPower(rightPower);
    }

    public void turnLeft(double speed){
        this.leftDriveBack.setPower(0);
        this.rightDriveBack.setPower(speed);
    }

    public void turnRight(double speed){
        this.leftDriveBack.setPower(speed);
        this.rightDriveBack.setPower(0);
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

}

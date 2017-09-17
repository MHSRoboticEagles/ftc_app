package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by sjeltuhin on 9/12/17.
 */

public class BasicBotConfig {
    public DcMotor leftDriveBack = null;
    public DcMotor rightDriveBack = null;


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
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDriveBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stop (){
        this.leftDriveBack.setPower(0);
        this.rightDriveBack.setPower(0);
    }

    public void move(double speed){
        this.leftDriveBack.setPower(speed);
        this.rightDriveBack.setPower(speed);
    }

    public void turnLeft(double speed){
        this.leftDriveBack.setPower(0);
        this.rightDriveBack.setPower(speed);
    }

    public void turnRight(double speed){
        this.leftDriveBack.setPower(speed);
        this.rightDriveBack.setPower(0);
    }

}

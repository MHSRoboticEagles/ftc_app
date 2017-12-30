/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.RevDoubleBot;
import org.firstinspires.ftc.teamcode.bots.RevSingleBot;
import org.firstinspires.ftc.teamcode.skills.ColorCracker;
import org.firstinspires.ftc.teamcode.skills.DetectedColor;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Linear Rev2", group="Robot9160")
//@Disabled
public class BasicLinearMode extends LinearOpMode {

    // Declare OpMode members.
    RevDoubleBot robot   = new RevDoubleBot();
    private ElapsedTime     runtime = new ElapsedTime();
    private ColorCracker jewelHunter = new ColorCracker();
    DetectedColor dc = DetectedColor.NONE;
    private boolean liftUpPressed = false;
    private boolean liftDownPressed = false;

    @Override
    public void runOpMode() {
        robot.init(this.hardwareMap);
        telemetry.addData("Status", "Initialized");
        jewelHunter.init(hardwareMap);

        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  -gamepad1.left_stick_x;

            double strife  =  -gamepad1.right_stick_x;

            boolean colorSignal = gamepad2.x;
            if (colorSignal){
                DetectedColor dc = jewelHunter.detectColor(telemetry, 0);
                telemetry.addData("Auto", "Color = %s", dc.name());
            }

            if (Math.abs(strife) > 0 ){
                if (strife < 0){
                    robot.strifeRight(Math.abs(strife));
                }
                else{
                    robot.strifeLeft(Math.abs(strife));
                }
            }
            else{
                robot.move(drive, turn);
            }

            //claws
            boolean freeze = gamepad2.b;
            if (freeze){
                robot.sqeezeClaw();
            }
            else{
                double servoPosition = gamepad2.left_trigger;
                robot.moveClaw(servoPosition);
            }

            //lift
            boolean liftUp = gamepad2.y;
            boolean liftDown = gamepad2.a;

            double liftPos  = robot.getLiftPos();
            if (liftUp && !liftUpPressed){
                robot.liftUp();
            }
            else if (liftDown && !liftDownPressed){
                robot.liftDown();
            }

            //make sure we stop the lift movement after a single push of the buttons
            if(liftUp && !liftUpPressed){
                liftUpPressed = true;
            }
            if (!liftUp){
                liftUpPressed = false;
            }

            if(liftDown && !liftDownPressed){
                liftDownPressed = true;
            }
            if (!liftDown){
                liftDownPressed = false;
            }

            //relic

            float valArm = gamepad2.right_stick_y;
            robot.moveArm(0.2, valArm, telemetry);

            float valElbow = gamepad2.right_stick_y;
            robot.moveElbow(0.2, valElbow, telemetry);


            boolean relicClawshut = robot.isRelicClawShut();
            if (!relicClawshut && gamepad1.a){
                robot.closeRelicClaw();
            }
            else if (relicClawshut && gamepad1.y){
                robot.openRelicClaw();
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Lift", "Lift Position: %.2f" + liftPos);
            telemetry.update();
        }
    }
}

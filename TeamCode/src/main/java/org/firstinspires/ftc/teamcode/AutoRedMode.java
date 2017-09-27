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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.bots.BasicBotConfig;
import org.firstinspires.ftc.teamcode.skills.CryptoColumn;
import org.firstinspires.ftc.teamcode.skills.ImageRecognition;


@Autonomous(name="AutoRed Straight", group ="Robot9160")
//@Disabled
public class AutoRedMode extends LinearOpMode {

    private boolean foundVuMark = false;

    BasicBotConfig robot = new BasicBotConfig();   // Use our standard robot configuration
    private ElapsedTime runtime = new ElapsedTime();
    ImageRecognition imageRecognition = new ImageRecognition();
    private static double TIME_CUT_OFF = 10.0;  //stop recognition at 10 sec. Then just guess.

    @Override public void runOpMode() {
        robot.init(hardwareMap);

        imageRecognition.init(hardwareMap);

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();
        //kick the jewel

        //do image recognition
        imageRecognition.start();
        runtime.reset();

        CryptoColumn result = CryptoColumn.None;

        while (opModeIsActive() && (!foundVuMark || runtime.seconds() < TIME_CUT_OFF)) {

            result = imageRecognition.track(telemetry);
            if (result != CryptoColumn.None){
                foundVuMark = true;
                switch (result){
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
                        break;
                }
            }
        }
    }

    //move to place the ball

    protected void moveToRight(){

        telemetry.addData("VuMark", "I am going to the right cell");
        robot.turnRight(0.4);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {

        }

        robot.stop();
    }

    protected void moveToLeft(){
        telemetry.addData("VuMark", "I am going to the left cell");
        robot.turnLeft(0.4);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {

        }

        robot.stop();
    }

    protected void moveToCenter(){

        telemetry.addData("VuMark", "I am going to the center cell");
        robot.move(0.6, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {

        }

        robot.stop();
    }

}

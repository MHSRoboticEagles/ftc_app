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
package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.RevDoubleBot;
import org.firstinspires.ftc.teamcode.gamefield.GameStats;
import org.firstinspires.ftc.teamcode.skills.ColorCracker;
import org.firstinspires.ftc.teamcode.skills.CryptoColumn;
import org.firstinspires.ftc.teamcode.skills.DetectedColor;
import org.firstinspires.ftc.teamcode.skills.ImageRecognition;


@Autonomous(name="AutoBlue Straight Rev", group ="Robot9160")
//@Disabled
public class AutoBlueRevModeStraight extends AutoBase {

    @Override public void runOpMode() {
        runAutoMode();
    }

    @Override
    protected void complete() {
        super.complete();
        completeStraight();
    }

    @Override
    protected void jewel(CryptoColumn column) {
        super.jewel(column);
        approach();
    }

    @Override
    protected void kick() {
        super.kick();
        blueKick();
    }

    protected void approach(){
        robot.encoderDrive(DRIVE_SPEED, 17.5, 17.5, 0, telemetry);
    }


    @Override
    protected void moveToRight(){
        telemetry.addData("Auto", "I am going to the right column");
        double moveTo = GameStats.CRYPTO_COL_WIDTH * 2.1;
        strafeToColumn(moveTo);
    }

    @Override
    protected void moveToLeft(){
        telemetry.addData("VuMark", "I am going to the left cell");
        double moveTo = GameStats.CRYPTO_COL_WIDTH*0.4;
        strafeToColumn(moveTo);
    }

    @Override
    protected void moveToCenter(){
        telemetry.addData("VuMark", "I am going to the center cell");
        double moveTo = GameStats.CRYPTO_COL_WIDTH/1.82 * 2;
        strafeToColumn(moveTo);
    }
}

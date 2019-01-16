package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.skills.Parrot;

@Autonomous(name="Move Test", group ="Robot15173")
//@Disabled
public class MoveTest extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        move(0.2, 10);
    }


}

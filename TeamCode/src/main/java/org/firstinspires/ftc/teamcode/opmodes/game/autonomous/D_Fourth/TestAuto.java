package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.D_Fourth;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.hardware.robot.Robot;

//Best Battery: 12.8 / 12.9

@Autonomous(name = "Test Auto")
@Disabled
public class TestAuto extends AutoControls {

    @Override
    public void runOpMode() throws InterruptedException {

        init(hardwareMap);
        alliance = 'b';
        waitForStart();

        performAction(0, 270, 6, 6, -1, -1, -1, -1, -1, -1, 0, 0);
        sleep(5000);
        performAction(15, 270, 6.5, 9, -1, -1, -1, -1, -1, -1, 0, 0);
        sleep(2500);
    }
}
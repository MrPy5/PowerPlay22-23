package org.firstinspires.ftc.teamcode.opmodes.testing;

import org.firstinspires.ftc.teamcode.opmodes.game.autonomous.D_Fourth.AutoControls;

public class EncoderDisabledTest extends AutoControls {

    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap);
        waitForStart();
        performActionTest(30, 0, 7, 10, -1, -1, -1, -1, -1, -1, 0, 0);
    }
}

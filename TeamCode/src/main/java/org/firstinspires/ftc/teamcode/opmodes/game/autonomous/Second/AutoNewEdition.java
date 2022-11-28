package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.Second;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.game.autonomous.Second.AutoControls;

@Autonomous(name = "New Auto")

public class AutoNewEdition extends AutoControls {
    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap);
        waitForStart();
        performAction(64, liftJunctionHighHeight, 50, turretRightDegrees, 50, 'n');


    }
}

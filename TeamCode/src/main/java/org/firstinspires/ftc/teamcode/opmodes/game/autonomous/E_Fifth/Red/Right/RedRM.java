package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.E_Fifth.Red.Right;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.game.autonomous.E_Fifth.RightAutoMid;

@Autonomous(name = "Red Right Medium", group = "Red", preselectTeleOp = "GAME TELEOP")

public class RedRM extends RightAutoMid {

    @Override
    public void runOpMode() throws InterruptedException {

        alliance = 'r';
        side = 'r';
        startAuto();
    }
}

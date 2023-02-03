package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.E_Fifth.Red.Right;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.game.autonomous.E_Fifth.RightAutoHigh;

@Autonomous(name = "Red Right High", group = "Red", preselectTeleOp = "GAME TELEOP")

public class RedRH extends RightAutoHigh {

    @Override
    public void runOpMode() throws InterruptedException {

        alliance = 'r';
        side = 'r';
        startAuto();
    }
}

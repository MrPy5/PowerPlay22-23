package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.E_Fifth.Red.Left;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.game.autonomous.E_Fifth.LeftAuto;

@Autonomous(name = "Red Left Medium", group = "Red", preselectTeleOp = "GAME TELEOP")

public class RedLM extends LeftAuto {

    @Override
    public void runOpMode() throws InterruptedException {

        alliance = 'r';
        side = 'l';
        startAuto();
    }
}

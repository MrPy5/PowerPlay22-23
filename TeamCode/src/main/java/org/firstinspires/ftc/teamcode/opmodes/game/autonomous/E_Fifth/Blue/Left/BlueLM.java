package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.E_Fifth.Blue.Left;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.game.autonomous.E_Fifth.LeftAuto;

@Autonomous(name = "Blue Left Medium", group = "Blue", preselectTeleOp = "GAME TELEOP")

public class BlueLM extends LeftAuto {

    @Override
    public void runOpMode() throws InterruptedException {

        alliance = 'b';
        side = 'l';
        startAuto();
    }
}

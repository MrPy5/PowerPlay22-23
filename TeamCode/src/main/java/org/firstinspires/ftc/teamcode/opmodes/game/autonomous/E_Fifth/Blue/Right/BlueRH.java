package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.E_Fifth.Blue.Right;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.game.autonomous.E_Fifth.RightAutoHigh;

@Autonomous(name = "Blue Right High", group = "Blue", preselectTeleOp = "GAME TELEOP")

public class BlueRH extends RightAutoHigh {

    @Override
    public void runOpMode() throws InterruptedException {

        alliance = 'b';
        startAuto();
    }
}

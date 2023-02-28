package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.E_Fifth.Blue.Right;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.game.autonomous.E_Fifth.RightAuto;
import org.firstinspires.ftc.teamcode.opmodes.game.autonomous.E_Fifth.RightAutoMid;

@Autonomous(name = "Blue Right Medium", group = "Blue", preselectTeleOp = "GAME TELEOP")

public class BlueRM extends RightAuto {

    @Override
    public void runOpMode() throws InterruptedException {

        alliance = 'b';
        side = 'r';
        startAuto();
    }
}

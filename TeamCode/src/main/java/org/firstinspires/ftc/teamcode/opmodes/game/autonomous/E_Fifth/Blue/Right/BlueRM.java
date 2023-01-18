package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.E_Fifth.Blue.Right;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.game.autonomous.E_Fifth.RightAutoMid;

@Autonomous(name = "Blue Right Medium", group = "Blue", preselectTeleOp = "GAME TELEOP")

public class BlueRM extends RightAutoMid {

    @Override
    public void runOpMode() throws InterruptedException {

        alliance = 'b';
        startAuto();
    }
}

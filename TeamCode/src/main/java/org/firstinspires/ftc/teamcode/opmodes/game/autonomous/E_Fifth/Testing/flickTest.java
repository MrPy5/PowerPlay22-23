package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.E_Fifth.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
;
import org.firstinspires.ftc.teamcode.opmodes.game.autonomous.E_Fifth.TestAuto;

@Autonomous(name = "flick test")

public class flickTest extends TestAuto {

    @Override
    public void runOpMode() throws InterruptedException {

        alliance = 'r';
        startAuto();
    }
}

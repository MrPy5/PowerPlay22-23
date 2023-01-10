package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.E_Fifth;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.hardware.robot.Robot;

@Autonomous(name = "ColorSensor Test")
@Disabled
public class TestAuto extends AutoControls{

    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap);
        alliance = 'b';
        waitForStart();

        performAction(24.75, 0, 12, 9, -1, 24, -1, 0, Robot.grabberServoHalfwayPos, 24, .5, 0, true);

    }
}

package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.E_Fifth;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.hardware.robot.Robot;

//Best Battery: 12.8 / 12.9
//Test

public abstract class TestAuto extends AutoControls {


    public void startAuto() {

        init(hardwareMap);
        waitForStart();

        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        performAction(25, 0, 6.5, 9, -1, 10, -1, 22, -1, 0, 0, 0, true, new double[] {-1, 0, 0});



    }
}

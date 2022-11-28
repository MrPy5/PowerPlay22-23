package org.firstinspires.ftc.teamcode.opmodes.game.autonomous;

import org.firstinspires.ftc.teamcode.hardware.robot.Robot;

public class AutoNewExample extends AutoControls {
    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap);

        Task lift = new Task(Robot.liftMotor, 500, 1, () -> Robot.frontLeft.getCurrentPosition() > 4000);

    }
}

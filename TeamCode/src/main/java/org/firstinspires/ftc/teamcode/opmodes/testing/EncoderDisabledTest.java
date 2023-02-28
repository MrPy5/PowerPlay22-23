package org.firstinspires.ftc.teamcode.opmodes.testing;

import org.firstinspires.ftc.teamcode.hardware.robot.Robot;
import org.firstinspires.ftc.teamcode.opmodes.game.autonomous.D_Fourth.AutoControls;

public class EncoderDisabledTest extends AutoControls {

    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap);
        waitForStart();
        performAction(24, 0, 7, 10, -1, -1, -1, -1, -1, -1, 0, 0);
        double initialPosition = getAverageOdometerPosition();
        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        double afterPosition = getAverageOdometerPosition();

        double odometerDifference = initialPosition - afterPosition;

        telemetry.addData("Initial Position: ", initialPosition);
        telemetry.addData("After Position: ", afterPosition);
        telemetry.update();

        double nextDrive = 24;
        double correctedDrive = nextDrive - odometerDifference;

        performAction(-correctedDrive, 0, 7, 10, -1, -1, -1, -1, -1, -1, 0, 0);

    }
}

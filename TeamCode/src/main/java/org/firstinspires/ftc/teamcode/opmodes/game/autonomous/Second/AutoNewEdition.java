package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.Second;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.game.autonomous.Second.AutoControls;

@Autonomous(name = "New Auto")

public class AutoNewEdition extends AutoControls {
    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap);
        //int pos = DetectAprilTags();

        ChangeGripperState(grabberServoClosedPos);
        performAction(40, liftJunctionMediumHeight, 35, turretRightDegrees, 22, 'n');
        performAction(0, liftJunctionMediumHeight - 4, 1, -1, -1, 'n');

        ChangeGripperState(grabberServoOpenPos);

        sleep(500);
        performAction(0, liftJunctionMediumHeight, 1, -1, 0, 'n');
        performAction(12, liftJunctionGroundHeight + 5.5, 8, turretForwardDegrees, 11, 'n');

        Turn(90);

        performAction(23.5, 5.5, 21, -1, 0, 'n');

        for (int i = 0; i < 2; i++) {

            ChangeGripperState(grabberServoClosedPos);
            sleep(500);
            performAction(0, liftJunctionMediumHeight, 0, -1, 0, 'n');
            performAction(-33.5, -1, 0, turretLeftDegrees, 11, 'n');
            performAction(0, liftJunctionMediumHeight - 4, 1, -1, 1, 'n');
            ChangeGripperState(grabberServoOpenPos);

            sleep(500);
            performAction(0, liftJunctionMediumHeight, 1, -1, 0, 'n');
            ChangeGripperState(grabberServoOpenPos);

            performAction(33.5, 5.5 - (i * 1.1), 22, turretForwardDegrees, 22, 'n');

        }

    }
}

package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.Second;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.game.autonomous.Second.AutoControls;

@Autonomous(name = "New Auto")

public class AutoNewEdition extends AutoControls {
    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap);
        int endParkingPosition = DetectAprilTags();
        //waitForStart();
        ChangeGripperState(grabberServoClosedPos);
        performAction(40, liftJunctionMediumHeight, 35, turretRightDegrees, 22, 'n');
        performAction(0, liftJunctionMediumHeight - 4, 1, -1, -1, 'n');

        ChangeGripperState(grabberServoOpenPos);

        sleep(500);
        performAction(0, liftJunctionMediumHeight, 1, -1, 0, 'n');
        performAction(12, liftJunctionGroundHeight + 5.5, 8, turretForwardDegrees, 11, 'n');

        Turn(87);

        ChangeGripperState(grabberServoHalfwayPos);
        performAction(24.5, 5.5, 21, -1, 0, 'n');


        ChangeGripperState(grabberServoClosedPos);
        sleep(500);
        performAction(0, liftJunctionMediumHeight, 0, -1, 0, 'n');
        performAction(-32.5, liftJunctionMediumHeight, 31, turretLeftDegrees, 11, 'n');
        performAction(0, liftJunctionMediumHeight - 4, 1, -1, 1, 'n');
        ChangeGripperState(grabberServoOpenPos);

        sleep(500);
        performAction(0, liftJunctionMediumHeight, 1, -1, 0, 'n');

        ChangeGripperState(grabberServoHalfwayPos);
        performAction(33.5, 5.5 - (0 * 1.3), 22, turretForwardDegrees, 22, 'n');


        ChangeGripperState(grabberServoClosedPos);
        sleep(500);
        performAction(0, liftJunctionMediumHeight, 0, -1, 0, 'n');
        performAction(-33.5, liftJunctionMediumHeight, 31, turretLeftDegrees, 11, 'n');
        performAction(0, liftJunctionMediumHeight - 4, 1, -1, 1, 'n');
        ChangeGripperState(grabberServoOpenPos);

        sleep(500);
        performAction(0, liftJunctionMediumHeight, 1, -1, 0, 'n');

        ChangeGripperState(grabberServoHalfwayPos);


        if (endParkingPosition == 3) {

            performAction(-13, -1, 0, -1, 0, 'n');
        }

        else if (endParkingPosition == 2) {
            performAction(13, -1, 0, -1, 0, 'n');
        }

        else if (endParkingPosition == 1) {

            performAction(33, -1, 0, -1, 0, 'n');


        }

        else {
            performAction(13, -1, 0, -1, 0, 'n');
        }

        Turn(180);
        performAction(0, 0, 0, 0, 0, 'n');



    }
}

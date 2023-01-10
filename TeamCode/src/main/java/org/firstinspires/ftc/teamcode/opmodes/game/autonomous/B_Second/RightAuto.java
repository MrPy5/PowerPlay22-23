package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.B_Second;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "**Right Auto Old")
@Disabled
public class RightAuto extends AutoControls {
    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap);
        int endParkingPosition = DetectAprilTags();
        //waitForStart();
        ChangeGripperState(grabberServoClosedPos);
        performAction(41, liftJunctionMediumHeight, 35, turretLeftDegrees, 22, 'n');
        performAction(0, liftJunctionMediumHeight - 4, 1, -1, -1, 'n');

        ChangeGripperState(grabberServoOpenPos);

        sleep(500);
        performAction(0, liftJunctionMediumHeight, 1, -1, 0, 'n');
        performAction(9.5, liftJunctionGroundHeight + 5.5, 8, turretForwardDegrees, 11, 'n');

        Turn(273);

        ChangeGripperState(grabberServoHalfwayPos);
        performAction(24.5, 5.5, 15, -1, 0, 'n');


        ChangeGripperState(grabberServoClosedPos);
        sleep(500);
        performAction(0, liftJunctionMediumHeight, 0, -1, 0, 'n');
        performAction(-32.5, liftJunctionMediumHeight, 31, turretRightDegrees, 11, 'n');
        performAction(0, liftJunctionMediumHeight - 4, 1, -1, 1, 'n');
        ChangeGripperState(grabberServoOpenPos);

        sleep(500);
        performAction(0, liftJunctionMediumHeight, 1, -1, 0, 'n');

        ChangeGripperState(grabberServoHalfwayPos);
        performAction(33.5, 5.5 - (1 * 1.2), 15, turretForwardDegrees, 22, 'n');


        ChangeGripperState(grabberServoClosedPos);
        sleep(500);
        performAction(0, liftJunctionMediumHeight, 0, -1, 0, 'n');
        performAction(-33, liftJunctionMediumHeight, 31, turretRightDegrees, 11, 'n');
        performAction(0, liftJunctionMediumHeight - 4, 1, -1, 1, 'n');
        ChangeGripperState(grabberServoOpenPos);

        sleep(500);
        performAction(0, liftJunctionMediumHeight, 1, -1, 0, 'n');

        ChangeGripperState(grabberServoClosedPos);


        if (endParkingPosition == 1) {

            performAction(-11.5, -1, 0, -1, 0, 'n');
        }

        else if (endParkingPosition == 2) {
            performAction(13, -1, 0, -1, 0, 'n');
        }

        else if (endParkingPosition == 3) {

            performAction(31.5, -1, 0, -1, 0, 'n');


        }

        else {
            performAction(13, -1, 0, -1, 0, 'n');
        }

        Turn(180);
        performAction(0, 0, 0, 0, 0, 'n');



    }
}

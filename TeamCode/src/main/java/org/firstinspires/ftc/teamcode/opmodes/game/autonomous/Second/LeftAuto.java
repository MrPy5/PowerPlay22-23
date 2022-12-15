package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.Second;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.robot.Robot;

@Autonomous(name = "Left Auto")

public class LeftAuto extends AutoControls {
    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap);
        //int endParkingPosition = DetectAprilTags();
        waitForStart();
        Robot.guideServo.setPosition(Robot.guideServoUp);
        //waitForStart();
        //Deliver first cone
        ChangeGripperState(grabberServoClosedPos);
        performAction(40, liftJunctionMediumHeight, 35, turretRightDegrees, 22, 'n');
        performAction(0, liftJunctionMediumHeight - 4, 1, -1, -1, 'n');

        ChangeGripperState(grabberServoOpenPos);

        //Return
        sleep(500);
        performAction(0, liftJunctionMediumHeight, 1, -1, 0, 'n');
        performAction(13, liftJunctionGroundHeight + 5.5, 8, turretForwardDegrees, 11, 'n');

        Turn(87);

        ChangeGripperState(grabberServoHalfwayPos);
        performAction(24.5, 5.5, 15, -1, 0, 'n');

        //Deliver Second Cone
        ChangeGripperState(grabberServoClosedPos);
        sleep(500);
        performAction(0, liftJunctionMediumHeight, 1, -1, 0, 'n');
        performAction(-32, liftJunctionMediumHeight, 31, turretLeftDegrees, 11, 'n');
        sleep(250);
        performAction(0, liftJunctionMediumHeight - 4, 1, -1, 1, 'n');
        ChangeGripperState(grabberServoOpenPos);

        //Return
        sleep(500);

        performAction(0, liftJunctionMediumHeight, 1, -1, 0, 'n');

        ChangeGripperState(grabberServoHalfwayPos);
        performAction(34, 5.5 - (1 * 1.3), 15, turretForwardDegrees, 22, 'n');

        //Deliver Third Cone
        ChangeGripperState(grabberServoClosedPos);
        sleep(500);
        performAction(0, liftJunctionMediumHeight, 1, -1, 0, 'n');
        performAction(-32, liftJunctionMediumHeight, 31, turretLeftDegrees, 11, 'n');
        sleep(250);
        performAction(0, liftJunctionMediumHeight - 4, 1, -1, 1, 'n');
        ChangeGripperState(grabberServoOpenPos);

        //Return
        sleep(500);

        performAction(0, liftJunctionMediumHeight, 1, -1, 0, 'n');

        ChangeGripperState(grabberServoHalfwayPos);
        performAction(34, 5.5 - (1 * 1.3), 15, turretForwardDegrees, 22, 'n');

        //Deliver Fourth Cone
        ChangeGripperState(grabberServoClosedPos);
        sleep(500);
        performAction(0, liftJunctionMediumHeight, 1, -1, 0, 'n');
        performAction(-32, liftJunctionMediumHeight, 31, turretLeftDegrees, 11, 'n');
        sleep(250);
        performAction(0, liftJunctionMediumHeight - 4, 1, -1, 1, 'n');
        ChangeGripperState(grabberServoOpenPos);

        sleep(500);

        performAction(0, liftJunctionMediumHeight, 1, -1, 0, 'n');

        ChangeGripperState(grabberServoClosedPos);

        /*
        if (endParkingPosition == 3) {

            performAction(-12.5, -1, 0, -1, 0, 'n');
        }

        else if (endParkingPosition == 2) {
            performAction(13, -1, 0, -1, 0, 'n');
        }

        else if (endParkingPosition == 1) {

            performAction(34, -1, 0, -1, 0, 'n');


        }

        else {
            performAction(13, -1, 0, -1, 0, 'n');
        }

        Turn(180);
        performAction(0, 0, 0, 0, 0, 'n');*/



    }
}

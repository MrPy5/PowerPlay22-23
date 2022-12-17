package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.Third;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.robot.Robot;

@Autonomous(name = "Right Auto")
public class RightAuto extends AutoControls{

    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap);
        //int endParkingPosition = DetectAprilTags();
        waitForStart();
        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        performAction(40, 0, 6.5, 10, Robot.liftJunctionMediumHeight, 37, Robot.turretLeftDegrees, 22, Robot.grabberServoOpenPos, 0, 0);

        performAction(12, 0, 3, 9, -1, 0, Robot.turretForwardDegrees, 12, -1, 0, 0);

        //Turn
        performAction(0, 273, 7, 7, -1, 0, -1, 0, -1, 0,0);
        //Drive to stack
        performAction(24, 273, 5, 11, Robot.liftJunctionGroundHeight + 4.25, 24, -1, 0, Robot.grabberServoHalfwayPos, 24, .5);

        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        sleep(500);
        //Lift 6 inches
        performAction(0, -1, 7, 0, Robot.liftJunctionGroundHeight + 6, 5, -1, 0, -1, 0, 0);

        // Go to pole + drop
        performAction(-34, 273, 6, 10, Robot.liftJunctionMediumHeight, 38, Robot.turretRightDegrees, 20, Robot.grabberServoOpenPos, 0, 0);
        sleep(500);

        //drive to stack
        performAction(35.5, 273, 7, 11, Robot.liftJunctionGroundHeight + 3, 30, Robot.turretForwardDegrees, 30, Robot.grabberServoHalfwayPos, 20, 0);

        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        sleep(500);
        performAction(0, -1, 7, 0, Robot.liftJunctionGroundHeight + 7, 1, -1, 0, -1, 0, 0);

        //Go to pole + drop
        performAction(-34.5, 273, 6, 10, Robot.liftJunctionMediumHeight, 38, Robot.turretRightDegrees, 20, Robot.grabberServoOpenPos, 0, 0);
        sleep(500);

        //Drive to stack
        performAction(35.5, 273, 7, 11, Robot.liftJunctionGroundHeight + 2, 30, Robot.turretForwardDegrees, 30, Robot.grabberServoHalfwayPos, 20, 0);

        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        sleep(500);
        //Lift 7 inches
        performAction(0, -1, 7, 0, Robot.liftJunctionGroundHeight + 7, 1, -1, 0, -1, 0, 0);

        //Go to pole + drop
        performAction(-34.5, 273, 6, 10, Robot.liftJunctionMediumHeight, 38, Robot.turretRightDegrees, 20, Robot.grabberServoOpenPos, 0, 0);
        sleep(500);

        //Drive to stack
        performAction(35.5, 273, 7, 11, Robot.liftJunctionGroundHeight + 1.25, 30, Robot.turretForwardDegrees, 30, Robot.grabberServoHalfwayPos, 20, 0);

        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        sleep(500);

        //Lift 7 inches
        performAction(0, -1, 7, 0, Robot.liftJunctionGroundHeight + 7, 1, -1, 0, -1, 0, 0);

        //Go to pole + drop
        performAction(-34.5, 273, 6, 10, Robot.liftJunctionMediumHeight, 38, Robot.turretRightDegrees, 20, Robot.grabberServoOpenPos, 0, 0);

        /*


        performAction(-37, 87, 7, 9);
        sleep(1000);


        performAction(37, 87, 7, 9);
        sleep(1000);

        performAction(-37, 87, 7, 9);
        sleep(1000);

        performAction(37, 87, 7, 9);
        sleep(1000);

        performAction(-37, 87, 7, 9);
        sleep(1000);

        performAction(37, 87, 7, 9);
        sleep(1000);

        performAction(-37, 87, 7, 9);
        sleep(1000);

        performAction(-35, 87, 7, 9);
        sleep(1000);*/
        
        
    }
}

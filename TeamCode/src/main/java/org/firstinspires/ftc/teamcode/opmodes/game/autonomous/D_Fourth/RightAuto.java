package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.D_Fourth;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.hardware.robot.Robot;

//Best Battery: 12.8 / 12.9

@Autonomous(name = "Right Auto")
public class RightAuto extends AutoControls {

    @Override
    public void runOpMode() throws InterruptedException {

        init(hardwareMap);
        alliance = 'b';
        int endParkingPosition = DetectAprilTags();
        //waitForStart();
        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        performAction(42, 0, 6.5, 9, Robot.liftJunctionMediumHeight, 37, Robot.turretLeftDegrees, 22, Robot.grabberServoOpenPos, 0, 0, 0);
        sleep(250);
        performAction(12, 0, 3, 9, -1, 0, Robot.turretForwardDegrees, 14, -1, 0, 0, 0);

        //Turn
        performAction(0, 271, 7, 0, -1, 0, -1, 0, -1, 0, 0, 0);
        //Drive to stack
        performAction(24.75, 271, 5, 11, Robot.liftJunctionGroundHeight + 4.25, 24, -1, 0, Robot.grabberServoHalfwayPos, 24, .5, 0);

        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        sleep(250);
        //Lift 6 inches
        performAction(0, -1, 7, 0, Robot.liftJunctionHighHeight, 5, -1, 0, -1, 0, 0, 20);

        // Go to pole + drop
        performAction(-36, 271, 6.5, 9, Robot.liftJunctionHighHeight, 38, Robot.turretLeftDegrees, 20, Robot.grabberServoOpenPos, 0, 0, 0);
        sleep(250);

        //drive to stack
        performAction(36, 270, 6.5, 8, Robot.liftJunctionGroundHeight + 3, 30, Robot.turretForwardDegrees, 35, Robot.grabberServoHalfwayPos, 20, 0, 0);

        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        sleep(250);
        performAction(0, -1, 7, 0, Robot.liftJunctionHighHeight, 1, -1, 0, -1, 0, 0, 20);

        //Go to pole + drop
        performAction(-36, 271, 6.5, 9, Robot.liftJunctionHighHeight, 38, Robot.turretLeftDegrees, 20, Robot.grabberServoOpenPos, 0, 0, 0);
        sleep(250);

        //Drive to stack
        performAction(36, 270, 6.5, 8, Robot.liftJunctionGroundHeight + 2, 30, Robot.turretForwardDegrees, 35, Robot.grabberServoHalfwayPos, 20, 0, 0);

        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        sleep(250);
        //Lift 7 inches
        performAction(0, -1, 7, 0, Robot.liftJunctionHighHeight, 1, -1, 0, -1, 0, 0, 20);

        //Go to pole + drop
        performAction(-36, 271, 6.5, 9, Robot.liftJunctionHighHeight, 38, Robot.turretLeftDegrees, 20, Robot.grabberServoOpenPos, 0, 0, 0);
        sleep(250);

        //Drive to stack
        performAction(36, 270, 6.5, 8, Robot.liftJunctionGroundHeight + 0.75, 30, Robot.turretForwardDegrees, 35, Robot.grabberServoHalfwayPos, 20, 0, 0);

        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        sleep(250);
        //Lift 7 inches
        performAction(0, -1, 7, 0, Robot.liftJunctionHighHeight, 1, -1, 0, -1, 0, 0, 20);

        //Go to pole + drop
        performAction(-36, 271, 6.5, 9, Robot.liftJunctionHighHeight, 38, Robot.turretLeftDegrees, 20, Robot.grabberServoOpenPos, 0, 0, 0);
        sleep(250);

        if (endParkingPosition == 1) {
            performAction(31.5, 180, 6, 11, Robot.liftJunctionGroundHeight, 25, Robot.turretForwardDegrees, 40, -1, -1, 1, 0);
        }
        if (endParkingPosition == 2) {
            performAction(8.5, 180, 5, 20, Robot.liftJunctionGroundHeight, 13, Robot.turretForwardDegrees, 13, -1, -1, 1, 0);
        }
        if (endParkingPosition == 3) {
            performAction(-14.5, 180, 5, 20, Robot.liftJunctionGroundHeight, 13, Robot.turretForwardDegrees, 13, -1, -1, 1, 0);
        }

        /*//Drive to stack
        performAction(35.5, 87, 7, 11, Robot.liftJunctionGroundHeight + 1.25, 30, Robot.turretForwardDegrees, 30, Robot.grabberServoHalfwayPos, 20, 0);

        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        sleep(500);

        //Lift 7 inches
        performAction(0, -1, 7, 0, Robot.liftJunctionGroundHeight + 7, 1, -1, 0, -1, 0, 0);

        //Go to pole + drop
        performAction(-34.5, 87, 6, 10, Robot.liftJunctionMediumHeight, 38, Robot.turretLeftDegrees, 20, Robot.grabberServoOpenPos, 0, 0);
        */


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

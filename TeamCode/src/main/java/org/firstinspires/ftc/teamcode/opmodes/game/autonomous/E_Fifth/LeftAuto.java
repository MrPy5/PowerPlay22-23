package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.E_Fifth;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.robot.Robot;

//Best Battery: 12.8 / 12.9
//Test

public abstract class LeftAuto extends AutoControls {

    double lastConeQuitTime = 23900;

    public void startAuto() {

        init(hardwareMap);



        int endParkingPosition = DetectAprilTags();
        if (endParkingPosition == 1) {
            lastConeQuitTime = lastConeQuitTime - 500;
        }

        cUMoveTimer.startTime();

        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        performAction(16.25, 0, 4, 6, Robot.liftJunctionLowHeight, 13, Robot.turretRightDegrees, 10, Robot.grabberServoOpenPos, 0, 0, 0, false, new double[] {1, 1, Robot.cURightOpenPos});

        //sleep(250);


        Robot.coneUprightLeftServo.setPosition(Robot.cULeftClosedPos);

        performAction(37.25, 0, 6.5, 9, Robot.liftJunctionMediumHeight, 47, Robot.turretForwardDegrees, 12, -1, 0, 0, 0, false, new double[] {37, 1, Robot.cURightClosedPos});



        performAction(24, 88, 5, 11, coneOneGrabHeight, 24, -1, 0, Robot.grabberServoHalfwayPos, 24, .5, 0, true, new double[] {-1, 0, 0});

        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        sleep(250);
        //Lift 6 inches
        performAction(0, -1, 7, 0, Robot.liftJunctionMediumHeight, 5, -1, 0, -1, 0, 0, 20, false, new double[] {-1, 0, 0});

        // Go to pole + drop
        performAction(-35, 88, 7, 10, Robot.liftJunctionMediumHeight, 38, Robot.turretLeftDegrees, 20, Robot.grabberServoOpenPos, 0, 0, 0, false, new double[] {-1, 0, 0});
        sleep(250);

        //drive to stack
        performAction(34.75, 86, 7, 8, coneTwoGrabHeight, 30, Robot.turretForwardDegrees, 35, Robot.grabberServoHalfwayPos, 20, 0, 0, true, new double[] {-1, 0, 0});

        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        sleep(250);
        performAction(0, -1, 7, 0, Robot.liftJunctionMediumHeight, 1, -1, 0, -1, 0, 0, 20, false, new double[] {-1, 0, 0});

        //Go to pole + drop
        performAction(-34.5, 88, 7, 10, Robot.liftJunctionMediumHeight, 38, Robot.turretLeftDegrees, 20, Robot.grabberServoOpenPos, 0, 0, 0, false, new double[] {-1, 0, 0});
        sleep(250);

        //Drive to stack
        performAction(34.75, 86, 7, 8, coneThreeGrabHeight, 30, Robot.turretForwardDegrees, 35, Robot.grabberServoHalfwayPos, 20, 0, 0, true, new double[] {-1, 0, 0});

        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        sleep(250);
        //Lift 7 inches
        performAction(0, -1, 7, 0, Robot.liftJunctionMediumHeight, 1, -1, 0, -1, 0, 0, 20, false, new double[] {-1, 0, 0});

        //Go to pole + drop
        performAction(-34.5, 88, 7, 10, Robot.liftJunctionMediumHeight, 38, Robot.turretLeftDegrees, 20, Robot.grabberServoOpenPos, 0, 0, 0, false, new double[] {-1, 0, 0});
        sleep(250);

        if (gameTimer.milliseconds() < lastConeQuitTime) {
            //Drive to stack
            performAction(34.75, 86, 7, 8, coneFourGrabHeight, 30, Robot.turretForwardDegrees, 35, Robot.grabberServoHalfwayPos, 20, 0, 0, true, new double[] {-1, 0, 0});

            Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
            sleep(250);
            //Lift 7 inches
            performAction(0, -1, 7, 0, Robot.liftJunctionMediumHeight, 1, -1, 0, -1, 0, 0, 20, false, new double[] {-1, 0, 0});

            //Go to pole + drop
            performAction(-34.5, 88, 7, 10, Robot.liftJunctionMediumHeight, 38, Robot.turretLeftDegrees, 20, Robot.grabberServoOpenPos, 0, 0, 0, false, new double[] {-1, 0, 0});
            sleep(250);
        }
        Robot.guideServo.setPosition(Robot.guideServoUp + 0.1);
        if (endParkingPosition != 1) {
            Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        }
        if (endParkingPosition == 1) {

            ZeroPowerToFloat();
            performAction(33, 90, 6, 11, Robot.liftJunctionGroundHeight, 25, Robot.turretForwardDegrees, 40, -1, -1, 1, 0, false, new double[] {-1, 0, 0});
        }
        if (endParkingPosition == 2) {
            performAction(8.5, 90, 5, 20, -1, 0, Robot.turretForwardDegrees, 8.5, -1, -1, 1, 0, false, new double[] {-1, 0, 0});
            performAction(0, 90, 0, 0, Robot.liftJunctionGroundHeight, 1, -1, 0, -1, -1, 0, 0, false, new double[] {-1, 0, 0});
        }
        if (endParkingPosition == 3) {
            performAction(-15, 90, 5, 20, -1, 0, Robot.turretForwardDegrees, 13, -1, -1, 1, 0, false, new double[] {-1, 0, 0});
            performAction(0, 90, 0, 0, Robot.liftJunctionGroundHeight, 1, -1, 0, -1, -1, 0, 0, false, new double[] {-1, 0, 0});
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

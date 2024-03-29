package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.E_Fifth;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.robot.Robot;

//Best Battery: 12.8 / 12.9



public abstract class RightAutoMid extends AutoControls {

    double lastConeQuitTime = 23900;

    public void startAuto() {

        init(hardwareMap);

        int endParkingPosition = 2;//DetectAprilTags();
        if (endParkingPosition == 3) {
            lastConeQuitTime = lastConeQuitTime - 500;
        }

        cUMoveTimer.startTime();

        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        performAction(18.5, 0, 4, 6, Robot.liftJunctionLowHeight, 8, Robot.turretLeftDegrees, 9, Robot.grabberServoOpenPos, 0, 0, 0, false, new double[] {6, 1, Robot.cURightFlickPos});


        Robot.coneUprightRightServo.setPosition(Robot.cURightClosedPos);


        performAction(32.5, 0, 7, 9, Robot.liftJunctionMediumHeight, 43, Robot.turretForwardDegrees, 14, -1, 0, 0, 0, false, new double[] {-1, 0,0});



        performAction(25, 270, 5.5, 11, coneOneGrabHeight, 24, -1, 0, Robot.grabberServoHalfwayPos, 24, .5, 0, true, new double[] {-1, 0, 0});
        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);

        sleep(250);
        //Lift 6 inches
        performAction(0, -1, 7, 0, Robot.liftJunctionMediumHeight, 5, -1, 0, -1, 0, 0, 31, false, new double[] {-1, 0, 0});

        // Go to pole + drop
        performAction(-36, 270, 7, 9, Robot.liftJunctionMediumHeight, 38, Robot.turretRightDegrees, 20, Robot.grabberServoOpenPos, 0, 0, 0, false, new double[] {-1, 0, 0});
        sleep(250);

        //drive to stack
        performAction(36.5, 267, 6.5, 8, coneTwoGrabHeight, 30, Robot.turretForwardDegrees, 35, Robot.grabberServoHalfwayPos, 20, 0, 0, true, new double[] {-1, 0, 0});

        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        sleep(250);
        performAction(0, -1, 7, 0, Robot.liftJunctionMediumHeight, 1, -1, 0, -1, 0, 0, 31, false, new double[] {-1, 0, 0});

        //Go to pole + drop
        performAction(-36, 270, 7, 9, Robot.liftJunctionMediumHeight, 38, Robot.turretRightDegrees, 20, Robot.grabberServoOpenPos, 0, 0, 0, false, new double[] {-1, 0, 0});
        sleep(250);

        //Drive to stack
        performAction(36.5, 267, 6.5, 8, coneThreeGrabHeight, 30, Robot.turretForwardDegrees, 35, Robot.grabberServoHalfwayPos, 20, 0, 0, true, new double[] {-1, 0, 0});

        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        sleep(250);
        //Lift 7 inches
        performAction(0, -1, 7, 0, Robot.liftJunctionMediumHeight, 1, -1, 0, -1, 0, 0, 31, false, new double[] {-1, 0, 0});

        //Go to pole + drop
        performAction(-36, 270, 7, 9, Robot.liftJunctionMediumHeight, 38, Robot.turretRightDegrees, 20, Robot.grabberServoOpenPos, 0, 0, 0, false, new double[] {-1, 0, 0});
        sleep(250);

        if (gameTimer.milliseconds() < lastConeQuitTime) {
            //Drive to stack
            performAction(36.5, 267, 6.5, 8, coneFourGrabHeight, 30, Robot.turretForwardDegrees, 35, Robot.grabberServoHalfwayPos, 20, 0, 0, true, new double[] {-1, 0, 0});

            Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
            sleep(250);
            //Lift 7 inches
            performAction(0, -1, 7, 0, Robot.liftJunctionMediumHeight, 1, -1, 0, -1, 0, 0, 31, false, new double[] {-1, 0, 0});

            //Go to pole + drop
            performAction(-36, 270, 7, 9, Robot.liftJunctionMediumHeight, 38, Robot.turretRightDegrees, 20, Robot.grabberServoOpenPos, 0, 0, 0, false, new double[] {-1, 0, 0});
            sleep(250);
        }
        Robot.guideServo.setPosition(Robot.guideServoUp + 0.1);
        if (endParkingPosition != 3) {
            Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        }
        if (endParkingPosition == 3) {
            ZeroPowerToFloat();
            performAction(34.5, 270, 6, 11, Robot.liftJunctionGroundHeight, 25, Robot.turretForwardDegrees, 40, -1, -1, 1, 0, false, new double[] {-1, 0, 0});
        }
        if (endParkingPosition == 2) {
            performAction(10, 270, 5, 20, Robot.liftJunctionGroundHeight, 3, Robot.turretForwardDegrees, 13, -1, -1, 1, 0, false, new double[] {-1, 0, 0});
        }
        if (endParkingPosition == 1) {
            performAction(-13.5, 270, 5, 20, Robot.liftJunctionGroundHeight, 13, Robot.turretForwardDegrees, 13, -1, -1, 1, 0, false, new double[] {-1, 0, 0});
        }

        /*//Drive to stack
        performAction(35.5, 87, 7, 11, Robot.liftJunctionGroundHeight + 1.25, 30, Robot.turretForwardDegrees, 30, Robot.grabberServoHalfwayPos, 20, 0);

        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        sleep(500);

        //Lift 7 inches
        performAction(0, -1, 7, 0, Robot.liftJunctionGroundHeight + 7, 1, -1, 0, -1, 0, 0);

        //Go to pole + drop
        performAction(-34.5, 87, 6, 10, Robot.liftJunctionMediumHeight, 38, Robot.turretRightDegrees, 20, Robot.grabberServoOpenPos, 0, 0);
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

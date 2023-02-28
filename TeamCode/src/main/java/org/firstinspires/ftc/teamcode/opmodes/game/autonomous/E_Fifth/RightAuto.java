package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.E_Fifth;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.robot.Robot;

//Best Battery: 13.5 - 13.9
//Test

public abstract class RightAuto extends AutoControls {

    double lastConeQuitTime = 25000;

    public void startAuto() {

        init(hardwareMap);



        int endParkingPosition = DetectAprilTags();

        if (endParkingPosition == 3) {
            lastConeQuitTime = lastConeQuitTime - 500;
        }

        if (endParkingPosition == 1) {
            lastConeQuitTime = lastConeQuitTime - 350;
        }

        cUMoveTimer.startTime();

        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        performAction(17, 0, 4, 8, Robot.liftJunctionLowHeight, 11, Robot.turretLeftDegrees, 14, Robot.grabberServoOpenPos, 0, 0.05, 0, false, new double[] {1, 1, Robot.cURightOpenPos});

        //sleep(200);


        Robot.coneUprightLeftServo.setPosition(Robot.cULeftClosedPos);

        performAction(36, 0, 6.5, 9, Robot.liftJunctionMediumHeight, 47, Robot.turretForwardDegrees, 12, -1, 0, 0, 0, false, new double[] {37, 1, Robot.cURightClosedPos});



        performAction(23.75, 271, 5.5, 11, coneOneGrabHeight, 24, -1, 1, Robot.grabberServoHalfwayPos, 24, 0, 0, true, new double[] {-1, 0, 0});

        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        sleep(200);
        //Lift 6 inches
        performAction(0, -1, 7, 0, Robot.liftJunctionMediumHeight, 5, -1, 0, -1, 0, 0, 20, false, new double[] {-1, 0, 0});

        // Go to pole + drop
        performAction(-34.25, 271, 7, 10, Robot.liftJunctionMediumHeight, 38, Robot.turretRightDegrees, 20, Robot.grabberServoOpenPos, 0, 0, 0, false, new double[] {-1, 0, 0});
        //sleep(100);

        //drive to stack
        performAction(34.5, 271, 7, 11, coneTwoGrabHeight, 30, Robot.turretForwardDegrees, 35, Robot.grabberServoHalfwayPos, 20, 0, 0, true, new double[] {-1, 0, 0});

        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        sleep(200);
        performAction(0, -1, 7, 0, Robot.liftJunctionMediumHeight, 1, -1, 0, -1, 0, 0, 20, false, new double[] {-1, 0, 0});

        //Go to pole + drop
        performAction(-34.25, 271, 7, 10, Robot.liftJunctionMediumHeight, 38, Robot.turretRightDegrees, 20, Robot.grabberServoOpenPos, 0, 0, 0, false, new double[] {-1, 0, 0});
        //sleep(100);

        //Drive to stack
        performAction(34.5, 271, 7, 11, coneThreeGrabHeight, 30, Robot.turretForwardDegrees, 35, Robot.grabberServoHalfwayPos, 20, 0, 0, true, new double[] {-1, 0, 0});

        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        sleep(200);
        //Lift 7 inches
        performAction(0, -1, 7, 0, Robot.liftJunctionMediumHeight, 1, -1, 0, -1, 0, 0, 20, false, new double[] {-1, 0, 0});

        //Go to pole + drop
        performAction(-34.25, 271, 7, 10, Robot.liftJunctionMediumHeight, 38, Robot.turretRightDegrees, 20, Robot.grabberServoOpenPos, 0, 0, 0, false, new double[] {-1, 0, 0});
        //sleep(100);

        if (gameTimer.milliseconds() < lastConeQuitTime) {
            //Drive to stack
            performAction(34.5, 271, 7, 11, coneFourGrabHeight, 30, Robot.turretForwardDegrees, 35, Robot.grabberServoHalfwayPos, 20, 0, 0, true, new double[] {-1, 0, 0});

            Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
            sleep(200);
            //Lift 7 inches
            performAction(0, -1, 7, 0, Robot.liftJunctionMediumHeight, 1, -1, 0, -1, 0, 0, 20, false, new double[] {-1, 0, 0});

            //Go to pole + drop
            performAction(-34.25, 271, 7, 10, Robot.liftJunctionMediumHeight, 38, Robot.turretRightDegrees, 20, Robot.grabberServoOpenPos, 0, 0, 0, false, new double[] {-1, 0, 0});
            //sleep(100);
        }

        if (gameTimer.milliseconds() < lastConeQuitTime) {
            //Drive to stack
            performAction(34.5, 271, 7, 11, coneFiveGrabHeight, 30, Robot.turretForwardDegrees, 35, Robot.grabberServoHalfwayPos, 20, 0, 0, true, new double[] {-1, 0, 0});

            Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
            sleep(200);
            //Lift 7 inches
            performAction(0, -1, 7, 0, Robot.liftJunctionMediumHeight, 1, -1, 0, -1, 0, 0, 20, false, new double[] {-1, 0, 0});

            //Go to pole + drop
            performAction(-34.25, 271, 7, 10, Robot.liftJunctionMediumHeight, 38, Robot.turretRightDegrees, 20, Robot.grabberServoOpenPos, 0, 0, 0, false, new double[] {-1, 0, 0});
            //sleep(100);
        }

        Robot.guideServo.setPosition(Robot.guideServoUp + 0.1);

        if (endParkingPosition == 3) {

            ZeroPowerToFloat();
            performAction(30, 270, 5, 11, Robot.liftJunctionGroundHeight, 25, Robot.turretForwardDegrees, 40, -1, -1, 0, 0.5, false, new double[] {-1, 0, 0});
        }
        if (endParkingPosition == 2) {
            performAction(8.5, 270, 5, 12, -1, 0, Robot.turretForwardDegrees, 8.5, -1, -1, 0, 0.5, false, new double[] {-1, 0, 0});

            Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);

            performAction(0, 270, 0, 0, Robot.liftJunctionGroundHeight, 1, -1, 0, -1, -1, 0, 0.5, false, new double[] {-1, 0, 0});
        }
        if (endParkingPosition == 1) {
            performAction(-15, 270, 4, 20, -1, 0, Robot.turretForwardDegrees, 13, -1, -1, 1, 0.5, false, new double[] {-1, 0, 0});

            Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);

            performAction(0, 270, 0, 0, Robot.liftJunctionGroundHeight, 1, -1, 0, -1, -1, 0, 0.5, false, new double[] {-1, 0, 0});
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
        sleep(2000);


        performAction(37, 87, 7, 9);
        sleep(2000);

        performAction(-37, 87, 7, 9);
        sleep(2000);

        performAction(37, 87, 7, 9);
        sleep(2000);

        performAction(-37, 87, 7, 9);
        sleep(2000);

        performAction(37, 87, 7, 9);
        sleep(2000);

        performAction(-37, 87, 7, 9);
        sleep(2000);

        performAction(-35, 87, 7, 9);
        sleep(2000);*/


    }
}

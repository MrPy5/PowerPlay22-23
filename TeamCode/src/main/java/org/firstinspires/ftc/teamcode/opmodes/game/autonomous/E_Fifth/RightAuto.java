package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.E_Fifth;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.robot.Robot;

//Best Battery: 13.5 - 13.9
//Test

public abstract class RightAuto extends AutoControls {

    double lastConeQuitTime = 25000;
    double nextDrive = 0;

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
        performAction(17.5, 0, 4, 7.5, Robot.liftJunctionLowHeight, 12, Robot.turretLeftDegrees, 14, Robot.grabberServoOpenPos, -2, 0.1, 0, false, new double[] {2, 1, Robot.cURightOpenPos});

        //sleep(200);



        performAction(35.5, 0, 6.5, 9, Robot.liftJunctionMediumHeight, 47, Robot.turretForwardDegrees, 12, -1, 0, 0, 0, false, new double[] {36, 1, Robot.cURightClosedPos});



        performAction(25.75, 270, 5.5, 11, coneOneGrabHeight, 24, -1, 1, Robot.grabberServoHalfwayPos, 24, 0, 0, true, new double[] {-1, 0, 0});

        /*Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        sleep(200);*/
        nextDrive = closeGrabberWithBounceback(34.75);
        //Lift 6 inches
        performAction(0, -1, 7, 0, Robot.liftJunctionMediumHeight, 5, -1, 0, -1, 0, 0, 20, false, new double[] {-1, 0, 0});

        // Go to pole + drop
        performAction(-nextDrive, 270, 7, 11, Robot.liftJunctionMediumHeight, 38, Robot.turretRightDegrees, 20, Robot.grabberServoOpenPos, -2, 0, 0, false, new double[] {-1, 0, 0});
        //sleep(100);

        //drive to stack
        performAction(34.75, 270, 7, 11, coneTwoGrabHeight, 30, Robot.turretForwardDegrees, 35, Robot.grabberServoHalfwayPos, 20, 0, 0, true, new double[] {-1, 0, 0});

        nextDrive = closeGrabberWithBounceback(34.75);

        performAction(0, -1, 7, 0, Robot.liftJunctionMediumHeight, 1, -1, 0, -1, 0, 0, 20, false, new double[] {-1, 0, 0});

        //Go to pole + drop
        performAction(-nextDrive, 270, 7, 11, Robot.liftJunctionMediumHeight, 38, Robot.turretRightDegrees, 20, Robot.grabberServoOpenPos, -2, 0, 0, false, new double[] {-1, 0, 0});
        //sleep(100);

        //Drive to stack
        performAction(34.75, 270, 7, 11, coneThreeGrabHeight, 30, Robot.turretForwardDegrees, 35, Robot.grabberServoHalfwayPos, 20, 0, 0, true, new double[] {-1, 0, 0});

        nextDrive = closeGrabberWithBounceback(34.75);
        //Lift 7 inches
        performAction(0, -1, 7, 0, Robot.liftJunctionMediumHeight, 1, -1, 0, -1, 0, 0, 20, false, new double[] {-1, 0, 0});

        //Go to pole + drop
        performAction(-nextDrive, 270, 7, 11, Robot.liftJunctionMediumHeight, 38, Robot.turretRightDegrees, 20, Robot.grabberServoOpenPos, -2, 0, 0, false, new double[] {-1, 0, 0});
        //sleep(100);

        if (gameTimer.milliseconds() < lastConeQuitTime) {
            //Drive to stack
            performAction(34.75, 270, 7, 11, coneFourGrabHeight, 30, Robot.turretForwardDegrees, 35, Robot.grabberServoHalfwayPos, 20, 0, 0, true, new double[] {-1, 0, 0});

            nextDrive = closeGrabberWithBounceback(34.75);
            //Lift 7 inches
            performAction(0, -1, 7, 0, Robot.liftJunctionMediumHeight, 1, -1, 0, -1, 0, 0, 20, false, new double[] {-1, 0, 0});

            //Go to pole + drop
            performAction(-nextDrive, 270, 7, 11, Robot.liftJunctionMediumHeight, 38, Robot.turretRightDegrees, 20, Robot.grabberServoOpenPos, -2, 0, 0, false, new double[] {-1, 0, 0});
            //sleep(100);
        }

        if (gameTimer.milliseconds() < lastConeQuitTime) {
            //Drive to stack
            performAction(34.75, 270, 7, 11, coneFiveGrabHeight, 30, Robot.turretForwardDegrees, 35, Robot.grabberServoHalfwayPos, 20, 0, 0, true, new double[] {-1, 0, 0});

            nextDrive = closeGrabberWithBounceback(34.75);
            //Lift 7 inches
            performAction(0, -1, 7, 0, Robot.liftJunctionMediumHeight, 1, -1, 0, -1, 0, 0, 20, false, new double[] {-1, 0, 0});

            //Go to pole + drop
            performAction(-nextDrive, 270, 7, 11, Robot.liftJunctionMediumHeight, 38, Robot.turretRightDegrees, 20, Robot.grabberServoOpenPos, -2, 0, 0, false, new double[] {-1, 0, 0});
            //sleep(100);
        }

        Robot.guideServo.setPosition(Robot.guideServoUp + 0.1);


        if (endParkingPosition == 3) {

            ZeroPowerToFloat();
            performAction(28, 90, 5, 11, Robot.liftJunctionGroundHeight, 25, Robot.turretForwardDegrees, 40, -1, -1, 0, 0.5, false, new double[] {-1, 0, 0});
        }
        if (endParkingPosition == 2) {
            performAction(9.25, 90, 4, 15, -1, 0, Robot.turretForwardDegrees, 8.5, Robot.grabberServoClosedPos, 7, 0, 0.5, false, new double[] {-1, 0, 0});


            performAction(0, 90, 0, 0, Robot.liftJunctionGroundHeight, 1, -1, 0, -1, -1, 0, 0.5, false, new double[] {-1, 0, 0});
        }
        if (endParkingPosition == 1) {
            performAction(-16, 90, 4, 20, -1, 0, Robot.turretForwardDegrees, 13, Robot.grabberServoClosedPos, 14, 1, 0.5, false, new double[] {-1, 0, 0});



            performAction(0, 90, 0, 0, Robot.liftJunctionGroundHeight, 1, -1, 0, -1, -1, 0, 0.5, false, new double[] {-1, 0, 0});
        }



    }

    public double closeGrabberWithBounceback(double nextDrive) {
        double initialPosition = getAverageOdometerPosition();
        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        sleep(200);
        double afterPosition = getAverageOdometerPosition();

        double odometerDifference = initialPosition - afterPosition;

        telemetry.addData("OdometeryDifference", odometerDifference);
        telemetry.update();

        return nextDrive - odometerDifference;
    }
}

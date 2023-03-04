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



        int endParkingPosition = 2;//DetectAprilTags();
        waitForStart();
        if (endParkingPosition == 3) {
            lastConeQuitTime = lastConeQuitTime - 500;
        }

        if (endParkingPosition == 1) {
            lastConeQuitTime = lastConeQuitTime - 350;
        }

        cUMoveTimer.startTime();

        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        performAction(18, 0, 4, 6, Robot.liftJunctionLowHeight, 13, Robot.turretLeftDegrees, 11, Robot.grabberServoOpenPos, 0, 0, 0, false, new double[] {2, 1, Robot.cURightOpenPos});

        //sleep(200);



        performAction(33, 0, 7, 5.5, Robot.liftJunctionMediumHeight, 47, Robot.turretForwardDegrees, 12, -1, 0, 0, 0, false, new double[] {31, 1, Robot.cURightClosedPos});



        performAction(25.75, 270, 5.25, 7, coneOneGrabHeight, 24, -1, 1, Robot.grabberServoHalfwayPos, 24, 0, 0, true, new double[] {-1, 0, 0});

        /*Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        sleep(200);*/
        nextDrive = closeGrabberWithBounceback(36.5);
        //Lift 7 inches
        performAction(0, -1, 7, 0, Robot.liftJunctionMediumHeight, 5, -1, 0, -1, 0, 0, 20, false, new double[] {-1, 0, 0});

        // Go to pole + drop
        performAction(-nextDrive, 271, 7, 7.5, Robot.liftJunctionMediumHeight, 38, Robot.turretRightDegrees, 20, Robot.grabberServoOpenPos, -2, 0, 0, false, new double[] {-1, 0, 0});
        //sleep(100);

        //drive to stack
        performAction(36.25, 270, 6.5, 5.5, coneTwoGrabHeight, 32, Robot.turretForwardDegrees, 35, Robot.grabberServoHalfwayPos, 20, 0, 0, true, new double[] {-1, 0, 0});

        nextDrive = closeGrabberWithBounceback(36.5);
        //Lift 7 inches
        performAction(0, -1, 7, 0, Robot.liftJunctionMediumHeight, 1, -1, 0, -1, 0, 0, 20, false, new double[] {-1, 0, 0});

        //Go to pole + drop
        performAction(-nextDrive, 271, 7, 6.5, Robot.liftJunctionMediumHeight, 38, Robot.turretRightDegrees, 20, Robot.grabberServoOpenPos, -2, 0, 0, false, new double[] {-1, 0, 0});
        //sleep(100);

        //Drive to stack
        performAction(36.25, 270, 6.5, 5.5, coneThreeGrabHeight, 32, Robot.turretForwardDegrees, 35, Robot.grabberServoHalfwayPos, 20, 0, 0, true, new double[] {-1, 0, 0});

        nextDrive = closeGrabberWithBounceback(36.5);
        //Lift 7 inches
        performAction(0, -1, 7, 0, Robot.liftJunctionMediumHeight, 1, -1, 0, -1, 0, 0, 20, false, new double[] {-1, 0, 0});

        //Go to pole + drop
        performAction(-nextDrive, 271, 7, 6.5, Robot.liftJunctionMediumHeight, 38, Robot.turretRightDegrees, 20, Robot.grabberServoOpenPos, -2, 0, 0, false, new double[] {-1, 0, 0});
        //sleep(100);

        if (gameTimer.milliseconds() < lastConeQuitTime) {
            //Drive to stack
            performAction(36.25, 270, 6.5, 5.5, coneFourGrabHeight, 32, Robot.turretForwardDegrees, 35, Robot.grabberServoHalfwayPos, 20, 0, 0, true, new double[] {-1, 0, 0});

            nextDrive = closeGrabberWithBounceback(36.5);
            //Lift 7 inches
            performAction(0, -1, 7, 0, Robot.liftJunctionMediumHeight, 1, -1, 0, -1, 0, 0, 20, false, new double[] {-1, 0, 0});

            //Go to pole + drop
            performAction(-nextDrive, 271, 7, 6.5, Robot.liftJunctionMediumHeight, 38, Robot.turretRightDegrees, 20, Robot.grabberServoOpenPos, -2, 0, 0, false, new double[] {-1, 0, 0});
            //sleep(100);
        }

        if (gameTimer.milliseconds() < lastConeQuitTime) {
            //Drive to stack
            performAction(36.25, 270, 6.5, 5.5, coneFiveGrabHeight, 32, Robot.turretForwardDegrees, 35, Robot.grabberServoHalfwayPos, 20, 0, 0, true, new double[] {-1, 0, 0});

            nextDrive = closeGrabberWithBounceback(36.5);
            //Lift 7 inches
            performAction(0, -1, 7, 0, Robot.liftJunctionMediumHeight, 1, -1, 0, -1, 0, 0, 20, false, new double[] {-1, 0, 0});

            //Go to pole + drop
            performAction(-nextDrive, 271, 7, 6.5, Robot.liftJunctionMediumHeight, 38, Robot.turretRightDegrees, 20, Robot.grabberServoOpenPos, -2, 0, 0, false, new double[] {-1, 0, 0});
            //sleep(100);
        }

        Robot.guideServo.setPosition(Robot.guideServoUp + 0.1);


        if (endParkingPosition == 3) {

            ZeroPowerToFloat();
            performAction(28, 270, 5, 7, Robot.liftJunctionGroundHeight, 25, Robot.turretForwardDegrees, 40, -1, -1, 0, 0.5, false, new double[] {-1, 0, 0});
        }
        if (endParkingPosition == 2) {
            performAction(10.25, 270, 3.5, 7, -1, 0, Robot.turretForwardDegrees, 8.5, Robot.grabberServoClosedPos, 7, 0, 0.5, false, new double[] {-1, 0, 0});


            performAction(0, 270, 0, 0, Robot.liftJunctionGroundHeight, 1, -1, 0, -1, -1, 0, 0.5, false, new double[] {-1, 0, 0});
        }
        if (endParkingPosition == 1) {
            performAction(-14.5, 270, 4, 7, -1, 0, Robot.turretForwardDegrees, 13, Robot.grabberServoClosedPos, 14, 1, 0.5, false, new double[] {-1, 0, 0});



            performAction(0, 270, 0, 0, Robot.liftJunctionGroundHeight, 1, -1, 0, -1, -1, 0, 0.5, false, new double[] {-1, 0, 0});
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

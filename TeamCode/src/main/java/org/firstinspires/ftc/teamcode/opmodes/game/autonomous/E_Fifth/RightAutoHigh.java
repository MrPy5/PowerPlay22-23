package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.E_Fifth;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.robot.Robot;

//Best Battery: 12.8 / 12.9



public abstract class RightAutoHigh extends AutoControls {

    public void startAuto() {

        init(hardwareMap);

        int endParkingPosition = DetectAprilTags();
        //waitForStart();
        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        performAction(43, 0, 7, 9, Robot.liftJunctionMediumHeight, 37, Robot.turretLeftDegrees, 22, Robot.grabberServoOpenPos, 0, 0, 0, false, new double[] {-1, 0, 0});
        sleep(250);
        performAction(9.5, 0, 3, 9, -1, 0, Robot.turretForwardDegrees, 14, -1, 0, 0, 0, false, new double[] {-1, 0, 0});

        //Turn
        //performAction(0, 267, 7, 0, -1, 0, -1, 0, -1, 0, 0, 0);
        //Drive to stack
        performAction(24.75, 266, 5.5, 11, coneOneGrabHeight, 24, -1, 0, Robot.grabberServoHalfwayPos, 24, .5, 0, true, new double[] {-1, 0, 0});

        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        sleep(250);
        //Lift 6 inches
        performAction(0, -1, 7, 0, Robot.liftJunctionHighHeight, 5, -1, 0, -1, 0, 0, 31, false, new double[] {-1, 0, 0});

        // Go to pole + drop
        performAction(-35.25, 267, 6.5, 10, Robot.liftJunctionHighHeight, 38, Robot.turretLeftDegrees, 20, Robot.grabberServoOpenPos, 0, 0, 0, false, new double[] {-1, 0, 0});
        sleep(250);

        //drive to stack
        performAction(36, 267, 6.5, 8, coneTwoGrabHeight, 30, Robot.turretForwardDegrees, 35, Robot.grabberServoHalfwayPos, 20, 0, 0, true, new double[] {-1, 0, 0});

        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        sleep(250);
        performAction(0, -1, 7, 0, Robot.liftJunctionHighHeight, 1, -1, 0, -1, 0, 0, 31, false, new double[] {-1, 0, 0});

        //Go to pole + drop
        performAction(-35.25, 267, 6.5, 10, Robot.liftJunctionHighHeight, 38, Robot.turretLeftDegrees, 20, Robot.grabberServoOpenPos, 0, 0, 0, false, new double[] {-1, 0, 0});
        sleep(250);

        //Drive to stack
        performAction(36, 267, 6.5, 8, coneThreeGrabHeight, 30, Robot.turretForwardDegrees, 35, Robot.grabberServoHalfwayPos, 20, 0, 0, true, new double[] {-1, 0, 0});

        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        sleep(250);
        //Lift 7 inches
        performAction(0, -1, 7, 0, Robot.liftJunctionHighHeight, 1, -1, 0, -1, 0, 0, 31, false, new double[] {-1, 0, 0});

        //Go to pole + drop
        performAction(-35.25, 267, 6.5, 10, Robot.liftJunctionHighHeight, 38, Robot.turretLeftDegrees, 20, Robot.grabberServoOpenPos, 0, 0, 0, false, new double[] {-1, 0, 0});
        sleep(250);
        if (gameTimer.milliseconds() < 25000) {

            //Drive to stack
            performAction(36, 267, 6.5, 8, coneFourGrabHeight, 30, Robot.turretForwardDegrees, 35, Robot.grabberServoHalfwayPos, 20, 0, 0, true, new double[] {-1, 0, 0});

            Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
            sleep(250);
            //Lift 7 inches
            performAction(0, -1, 7, 0, Robot.liftJunctionHighHeight, 1, -1, 0, -1, 0, 0, 31, false, new double[] {-1, 0, 0});

            //Go to pole + drop
            performAction(-35.25, 267, 6.5, 10, Robot.liftJunctionHighHeight, 38, Robot.turretLeftDegrees, 20, Robot.grabberServoOpenPos, 0, 0, 0, false, new double[] {-1, 0, 0});
            sleep(250);
        }

        if (endParkingPosition == 3) {
            ZeroPowerToFloat();
            performAction(31.5, 270, 6, 11, Robot.liftJunctionGroundHeight, 25, Robot.turretForwardDegrees, 40, -1, -1, 1, 0, false, new double[] {-1, 0, 0});
        }
        if (endParkingPosition == 2) {
            performAction(8.5, 270, 5, 20, Robot.liftJunctionGroundHeight, 13, Robot.turretForwardDegrees, 13, -1, -1, 1, 0, false, new double[] {-1, 0, 0});
        }
        if (endParkingPosition == 1) {
            performAction(-14.5, 270, 5, 20, Robot.liftJunctionGroundHeight, 13, Robot.turretForwardDegrees, 13, -1, -1, 1, 0, false, new double[] {-1, 0, 0});
        }



    }
}

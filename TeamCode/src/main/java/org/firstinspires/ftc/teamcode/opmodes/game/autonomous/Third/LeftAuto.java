package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.Third;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.robot.Robot;

@Autonomous(name = "New Auto")
public class LeftAuto extends AutoControls{

    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap);
        //int endParkingPosition = DetectAprilTags();
        waitForStart();
        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        performAction(40, 0, 7, 9, Robot.liftJunctionMediumHeight - 2, 35, Robot.turretRightDegrees, 22, Robot.grabberServoOpenPos, .3);

        performAction(12, 0, 3, 9, -1, 0, Robot.turretForwardDegrees, 12, -1, 0);

        //Turn
        performAction(0, 87, 7, 9, -1, 0, -1, 0, -1, 0);

        performAction(24, 87, 5.5, 9, Robot.liftJunctionGroundHeight + 4.25, 24, -1, 0, Robot.grabberServoHalfwayPos, 24);

        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        sleep(500);
        //Lift lift 10 inches
        performAction(0, 87, 7, 9, Robot.liftJunctionGroundHeight + 12, 5, -1, 0, -1, 0);

        // Go to pole
        performAction(-34, 87, 7, 10, Robot.liftJunctionMediumHeight, 38, Robot.turretLeftDegrees, 20, Robot.grabberServoOpenPos, 0.3);
        sleep(500);
        //Drop 2
        performAction(35.5, 87, 7, 10, Robot.liftJunctionGroundHeight + 3, 10, Robot.turretForwardDegrees, 30, Robot.grabberServoHalfwayPos, 20);

        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        sleep(500);
        performAction(0, 87, 7, 9, Robot.liftJunctionGroundHeight + 12, 1, -1, 0, -1, 0);

        performAction(-34.5, 87, 7, 10, Robot.liftJunctionMediumHeight, 38, Robot.turretLeftDegrees, 20, Robot.grabberServoOpenPos, 0.3);

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

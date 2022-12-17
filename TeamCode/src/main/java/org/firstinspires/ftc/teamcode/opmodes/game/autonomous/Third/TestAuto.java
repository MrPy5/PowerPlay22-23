package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.Third;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.hardware.robot.Robot;

@Autonomous(name = "Test Auto")
@Disabled
public class TestAuto extends AutoControls{

    @Override
    public void runOpMode() throws InterruptedException {

        init(hardwareMap);
        alliance = 'b';
        //int endParkingPosition = DetectAprilTags();
        waitForStart();
        performAction(0, 90, 5, 13, -1, -1, -1, -1, -1, -1, 0);
        performAction(4, 90, 5, 13, -1, 13, -1, 13, -1, -1, 1);
            //performAction(0, 180, 7, 11, -1, -1, -1, -1, -1, -1, 0);

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

//Checklist
/*

-180 turn for left
-


 */


package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.robot.Robot;


@TeleOp(name = "GAME TELEOP")

public class MotorTestWithDpad extends LinearOpMode {


    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap);
        waitForStart();


        waitForStart();


        while (opModeIsActive()) {
            Robot.frontLeft.setPower(gamepad1.left_trigger > 0.0001 ? 1 : 0);
            Robot.frontRight.setPower(gamepad1.right_trigger > 0.0001 ? 1 : 0);
            Robot.backLeft.setPower(gamepad1.left_bumper ? 1 : 0);
            Robot.backRight.setPower(gamepad1.right_bumper ? 1 : 0);
        }
    }
}


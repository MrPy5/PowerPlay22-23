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

    double triggerSensitivity = 0.01;

    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap);
        waitForStart();


        waitForStart();


        while (opModeIsActive()) {
            if (gamepad1.left_trigger > triggerSensitivity) {
                Robot.frontLeft.setPower(.3);
                telemetry.addData("FrontLeft:", Robot.frontLeft.getCurrentPosition());
            } else {
                Robot.frontLeft.setPower(0);
            }
            if (gamepad1.right_trigger > triggerSensitivity) {
                Robot.frontRight.setPower(.3);
                telemetry.addData("FrontRight:", Robot.frontRight.getCurrentPosition());
            } else {
                Robot.frontRight.setPower(0);
            }
            if (gamepad1.left_bumper) {
                Robot.backLeft.setPower(.3);
                telemetry.addData("BackLeft:", Robot.backLeft.getCurrentPosition());
            } else {
                Robot.backLeft.setPower(0);
            }
            if (gamepad1.right_bumper) {
                Robot.backRight.setPower(.3);
                telemetry.addData("BackRight:", Robot.backRight.getCurrentPosition());
            } else {
                Robot.backRight.setPower(0);
            }
            telemetry.update();
        }
    }
}


//Checklist
/*

-180 turn for left
-


 */


package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.robot.Robot;


@TeleOp(name = "Trigger/Bumper Motor/Encoder test")
@Disabled
public class MotorTestWithDpad extends LinearOpMode {

    double triggerSensitivity = 0.01;

    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap, true);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_trigger > triggerSensitivity) {
                robot.frontLeft.setPower(.3);
                telemetry.addData("FrontLeft:", robot.frontLeft.getCurrentPosition());
            } else {
                robot.frontLeft.setPower(0);
            }
            if (gamepad1.right_trigger > triggerSensitivity) {
                robot.frontRight.setPower(.3);
                telemetry.addData("FrontRight:", robot.frontRight.getCurrentPosition());
            } else {
                robot.frontRight.setPower(0);
            }
            if (gamepad1.left_bumper) {
                robot.backLeft.setPower(.3);
                telemetry.addData("BackLeft:", robot.backLeft.getCurrentPosition());
            } else {
                robot.backLeft.setPower(0);
            }
            if (gamepad1.right_bumper) {
                robot.backRight.setPower(.3);
                telemetry.addData("BackRight:", robot.backRight.getCurrentPosition());
            } else {
                robot.backRight.setPower(0);
            }
            telemetry.update();
        }
    }
}


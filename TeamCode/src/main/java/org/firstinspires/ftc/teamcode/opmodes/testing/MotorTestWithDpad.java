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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.robot.Robot;


@TeleOp(name = "Trigger/Bumper Motor/Encoder test")
public class MotorTestWithDpad extends LinearOpMode {

    double triggerSensitivity = 0.01;

    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap, true);
        double speed = .3;

        ElapsedTime dpad = new ElapsedTime();
        dpad.startTime();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_trigger > triggerSensitivity) {
                Robot.frontLeft.setPower(speed);
            } else {
                Robot.frontLeft.setPower(0);
            }
            if (gamepad1.right_trigger > triggerSensitivity) {
                Robot.frontRight.setPower(speed);
            } else {
                Robot.frontRight.setPower(0);
            }
            if (gamepad1.left_bumper) {
                Robot.backLeft.setPower(speed);
            } else {
                Robot.backLeft.setPower(0);
            }
            if (gamepad1.right_bumper) {
                Robot.backRight.setPower(speed);
            } else {
                Robot.backRight.setPower(0);
            }

            if (gamepad1.dpad_up) {
                    speed += 0.01;
            }

            if (gamepad1.dpad_down) {
                    speed -= 0.01;

            }


            if (gamepad1.touchpad_finger_1 && gamepad1.touchpad_finger_2) {
                Robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                Robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }

            telemetry.addData("FrontLeft: ", Robot.frontLeft.getCurrentPosition());

            telemetry.addData("FrontRight: ", Robot.frontRight.getCurrentPosition());

            telemetry.addData("BackLeft: ", Robot.backLeft.getCurrentPosition());

            telemetry.addData("BackRight: ", Robot.backRight.getCurrentPosition());

            telemetry.addData("Speed: ", speed);

            telemetry.update();
        }
    }
}


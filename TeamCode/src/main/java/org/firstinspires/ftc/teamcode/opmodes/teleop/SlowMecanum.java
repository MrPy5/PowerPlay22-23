package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.robot.Robot;


@TeleOp
public class SlowMecanum extends LinearOpMode {

    Robot robot = new Robot();

    @Override
    public void runOpMode() {

        waitForStart();

        while (opModeIsActive()) {



            double leftStickY = gamepad1.left_stick_y;
            double leftStickX = gamepad1.left_stick_x;
            double rightStickX = gamepad1.right_stick_x;

            double wheelPower;
            double stickAngleRadians;
            double rightX;
            double lfPower;
            double rfPower;
            double lrPower;
            double rrPower;

            wheelPower = Math.hypot(leftStickX, leftStickY);
            if (wheelPower > .05) {
                wheelPower = (.8 * Math.pow(wheelPower, 2) + .2) / 2;
            }
            stickAngleRadians = Math.atan2(leftStickY, leftStickX);

            stickAngleRadians = stickAngleRadians - Math.PI / 4; //adjust by 45 degrees

            double sinAngleRadians = Math.sin(stickAngleRadians);
            double cosAngleRadians = Math.cos(stickAngleRadians);
            double factor = 1 / Math.max(Math.abs(sinAngleRadians), Math.abs(cosAngleRadians));

            rightX = rightStickX * .5;

            lfPower = wheelPower * cosAngleRadians * factor + rightX;
            rfPower = wheelPower * sinAngleRadians * factor - rightX;
            lrPower = wheelPower * sinAngleRadians * factor + rightX;
            rrPower = wheelPower * cosAngleRadians * factor - rightX;

            robot.backLeft.setPower(lrPower);
            robot.backRight.setPower(rrPower);
            robot.frontLeft.setPower(lfPower);
            robot.frontRight.setPower(rfPower);

            //telemetry.addData("RL Servo: ", tapeMeasureRLServoPosition);
            //telemetry.addData("UD Servo: ", tapeMeasureUDServoPosition);
            telemetry.addData("LeftStickX", gamepad1.left_stick_x);
            telemetry.addData("LeftStickY", gamepad1.left_stick_y);
            telemetry.addData("RightStickX", gamepad1.right_stick_x);
            telemetry.addData("RightStickY", gamepad1.right_stick_y);
            telemetry.update();
        }
    }
}

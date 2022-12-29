package org.firstinspires.ftc.teamcode.opmodes.outreach;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.robot.Robot;

@TeleOp(name = "Outreach")
public class ScienceCenter extends LinearOpMode {


    public double slowfactor = 0.4;
    public double grabberCurrentPos = Robot.grabberServoOpenPos;
    public boolean triggerReleased = false;
    public boolean liftReleased = false;
    public double liftCurrentPos = 0;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, true);

        Robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Robot.grabberServo.setPosition(Robot.grabberServoOpenPos);
        waitForStart();

        while (opModeIsActive()) {

            double trigger = gamepad1.right_trigger;
            double liftTrigger = gamepad1.left_trigger;

            double leftStickY = gamepad1.left_stick_y * -1;
            double leftStickX = gamepad1.left_stick_x;
            double rightStickX = gamepad1.right_stick_x;

            double wheelPower;
            double stickAngleRadians;
            double rightX;
            double lfPower;
            double rfPower;
            double lrPower;
            double rrPower;


            if (trigger > Robot.triggerSensitivity && triggerReleased) {
                if (grabberCurrentPos == Robot.grabberServoOpenPos) {
                    grabberCurrentPos = Robot.grabberServoClosedPos;
                    Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);

                }
                else {
                    grabberCurrentPos = Robot.grabberServoOpenPos;
                    Robot.grabberServo.setPosition(Robot.grabberServoOpenPos);

                }
                triggerReleased = false;
            }
            if (trigger < Robot.triggerSensitivity) {
                triggerReleased = true;
            }

            if (liftTrigger > Robot.triggerSensitivity && liftReleased) {
                if (liftCurrentPos == 0) {
                    liftCurrentPos = Robot.liftJunctionMediumHeight;
                    Robot.liftMotor.setTargetPosition((int) (Robot.liftJunctionMediumHeight * Robot.liftTicksPerInch));
                    Robot.liftMotor.setPower(Robot.liftSpeedUp / 2);
                }
                else {
                    liftCurrentPos = 0;
                    Robot.liftMotor.setTargetPosition((int) (Robot.liftJunctionGroundHeight * Robot.liftTicksPerInch));
                    Robot.liftMotor.setPower(Robot.liftSpeedUp / 2);


                }
                liftReleased = false;
            }
            if (liftTrigger < Robot.triggerSensitivity) {
                liftReleased = true;
            }

            if (Robot.liftMotor.isBusy() == false && liftCurrentPos == 0) {
                Robot.liftMotor.setTargetPosition(0);
            }
            //DRIVING
            wheelPower = Math.hypot(leftStickX, leftStickY);
            if (wheelPower > .02) {
                wheelPower = (.8 * wheelPower + .2) * slowfactor;
            }


            stickAngleRadians = Math.atan2(leftStickY, leftStickX);

            stickAngleRadians = stickAngleRadians - Math.PI / 4; //adjust by 45 degrees

            double sinAngleRadians = Math.sin(stickAngleRadians);
            double cosAngleRadians = Math.cos(stickAngleRadians);
            double factor = 1 / Math.max(Math.abs(sinAngleRadians), Math.abs(cosAngleRadians));

            rightX = rightStickX * slowfactor * .8;

            lfPower = wheelPower * cosAngleRadians * factor + rightX;
            rfPower = wheelPower * sinAngleRadians * factor - rightX;
            lrPower = wheelPower * sinAngleRadians * factor + rightX;
            rrPower = wheelPower * cosAngleRadians * factor - rightX;

            Robot.backLeft.setPower(lrPower);
            Robot.backRight.setPower(rrPower);
            Robot.frontLeft.setPower(lfPower);
            Robot.frontRight.setPower(rfPower);


            telemetry.update();
        }
    }
}

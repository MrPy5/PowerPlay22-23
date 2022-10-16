package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.robot.Robot;


@TeleOp

public class TeleopFirst extends LinearOpMode {


    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap);
        waitForStart();


        double triggerSensitivity = 0.01;

        double slowfactor = 1;
        boolean slow_mode = false;
        boolean slowmoTriggerReleased = true;


        //---------------------------------------------------------------//
        //GRABBER SERVO VARIABLES

        double grabberServoClosedPos = 0.3;
        double grabberServoOpenPos = 0.7;
        double grabberServoCurrentPos = 0.3;
        boolean grabberTriggerReleased = true;


        //---------------------------------------------------------------//
        //GRABBER TURRET VARIABLES

        double turretSpeed = 0.25;
        int turretPosCurrentTicks = 0;
        int turretCloseToZero = 70;

        //         0
        //      |     |
        //    3    ^     1
        //      |     |
        //         2

        int forwardTurretPosTicks = 0;
        int rightTurretPosTicks = 696;
        int leftTurretPosTicks = -696;
        int backTurretPosTicks = 1393;

        int targetTurretPos = 0;


        //---------------------------------------------------------------//
        //LIFT VARIABLES

        int liftPosCurrentTicks = 0;
        double liftSpeed = 0.5;


        //   >____
        //    3  |  High
        //    2  |  Medium
        //    1  |  Low
        //    0  |  Ground
        //


        int ticksPerRevolutionOrbital = 537;

        int groundLiftPosTicks = 0;
        int lowLiftPosTicks = ticksPerRevolutionOrbital * 2;
        int mediumLiftPosTicks = ticksPerRevolutionOrbital * 3;
        int highLiftPosTicks = ticksPerRevolutionOrbital * 4;

        int targetLiftPos = 0;
        int liftMinHeightForTurning = lowLiftPosTicks;


        Robot.gripperServo.setPosition(grabberServoOpenPos);


        while (opModeIsActive()) {

            //---------------------------------------------------------------//
            //GAMEPAD2 CONTROLS
            double grabberTrigger = gamepad2.right_trigger;

            boolean turretPosForwardButton = gamepad2.dpad_up;
            boolean turretPosBackButton = gamepad2.dpad_down;
            boolean turretPosLeftButton = gamepad2.dpad_left;
            boolean turretPosRightButton = gamepad2.dpad_right;

            boolean liftPosGroundButton = gamepad2.x;
            boolean liftPosLowButton = gamepad2.a;
            boolean liftPosMediumButton = gamepad2.b;
            boolean liftPosHighButton = gamepad2.y;


            //---------------------------------------------------------------//
            //GAMEPAD1 CONTROLS

            double slowmoTrigger = gamepad1.right_trigger;

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


            //-----------------------------------------------------------//
            //SLO-MO CODE

            if (slowmoTrigger > triggerSensitivity) {
                if (slowmoTriggerReleased) {
                    if (slow_mode == false) {
                        slowfactor = 0.3;
                        slow_mode = true;
                    } else {
                        slowfactor = 1;
                        slow_mode = false;
                    }
                    slowmoTriggerReleased = false;
                }
            } else {
                slowmoTriggerReleased = true;
            }

            //---------------------------------------------------------------//
            //GRABBER SERVO CODE

            if (grabberTrigger > triggerSensitivity) {
                if (grabberTriggerReleased) {
                    if (grabberServoCurrentPos == grabberServoOpenPos) {
                        grabberServoCurrentPos = grabberServoClosedPos;
                    } else {
                        grabberServoCurrentPos = grabberServoOpenPos;
                    }
                    Robot.gripperServo.setPosition(grabberServoCurrentPos);
                    grabberTriggerReleased = false;
                }
            } else {
                grabberTriggerReleased = true;
            }


            //---------------------------------------------------------------//
            //TURRET CODE

            if (turretPosForwardButton) {
                targetTurretPos = forwardTurretPosTicks;
            }
            if (turretPosLeftButton) {
                targetTurretPos = leftTurretPosTicks;
            }
            if (turretPosRightButton) {
                targetTurretPos = rightTurretPosTicks;
            }
            if (turretPosBackButton) {
                targetTurretPos = backTurretPosTicks;
            }

            liftPosCurrentTicks = Robot.liftMotor.getCurrentPosition();

            if (liftPosCurrentTicks > liftMinHeightForTurning) {
                Robot.turretMotor.setTargetPosition(targetTurretPos);
                Robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Robot.turretMotor.setPower(liftSpeed * slowfactor);
            }

            //----------------------------------------------------------//
            //LIFT MOTOR

            if (liftPosGroundButton) {
                targetLiftPos = groundLiftPosTicks;
            }

            if (liftPosLowButton) {
                targetLiftPos = lowLiftPosTicks;
            }

            if (liftPosMediumButton) {
                targetLiftPos = mediumLiftPosTicks;
            }
            if (liftPosHighButton) {
                targetLiftPos = highLiftPosTicks;
            }

            turretPosCurrentTicks = Robot.turretMotor.getCurrentPosition();

            if (liftPosCurrentTicks < targetLiftPos || Math.abs(turretPosCurrentTicks) > turretCloseToZero || targetLiftPos > liftMinHeightForTurning) {
                Robot.liftMotor.setTargetPosition(targetLiftPos);
                Robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Robot.liftMotor.setPower(liftSpeed * slowfactor);
            }


            /*if (!robot.liftMotor.isBusy()) {
                robot.liftMotor.setPower(0);
                robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }*/


            //---------------------------------------------------------//
            //DRIVING CODE

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


            //---------------------------------------------------------------------//
            //TELEMETRY CODE


            telemetry.update();

        }
    }
}


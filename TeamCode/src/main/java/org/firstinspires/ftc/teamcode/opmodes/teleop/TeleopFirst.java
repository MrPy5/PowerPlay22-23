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

        double grabberServoClosedPos = 0.22;
        double grabberServoOpenPos = 0.7;
        double grabberServoCurrentPos = 0.3;
        boolean grabberTriggerReleased = true;


        //---------------------------------------------------------------//
        //TURRET VARIABLES

        double turretSpeed = 0.25;
        int turretPosCurrentTicks = 0;
        int turretCloseToZero = 70;

        int forwardTurretPosTicks = 0;
        int rightTurretPosTicks = 696;
        int leftTurretPosTicks = -696;
        int backTurretPosTicks = 1393;

        int targetTurretPos = 0;
        int prevTargetTurretPos = 0;


        //---------------------------------------------------------------//
        //LIFT VARIABLES

        int liftPosCurrentTicks = 0;
        double liftSpeed = 1;

        int ticksPerRevolutionOrbital = 537;

        int groundLiftPosTicks = 0;
        int lowLiftPosTicks = ticksPerRevolutionOrbital * 2;
        int mediumLiftPosTicks = ticksPerRevolutionOrbital * 3;
        int highLiftPosTicks = ticksPerRevolutionOrbital * 4;

        int targetLiftPos = 0;
        int prevTargetLiftPos = 0;
        int liftMinHeightForTurning = lowLiftPosTicks;


        //---------------------------------------------------------//
        //DRIVING CODE VARIABLES
        double sinAngleRadians;
        double cosAngleRadians;
        double factor;
        double wheelPower;
        double stickAngleRadians;
        double rightX;
        double lfPower;
        double rfPower;
        double lrPower;
        double rrPower;


        //---------------------------------------------------------//
        //INIT SERVOS
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

            //---------------------------------------------------------------//
            //READ HARDWARE VALUES
            liftPosCurrentTicks = Robot.liftMotor.getCurrentPosition();
            turretPosCurrentTicks = Robot.turretMotor.getCurrentPosition();


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

            if (liftPosCurrentTicks > liftMinHeightForTurning) {
                if (targetTurretPos != prevTargetTurretPos) {
                    Robot.turretMotor.setTargetPosition(targetTurretPos);
                    Robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Robot.turretMotor.setPower(turretSpeed);
                    prevTargetTurretPos = targetTurretPos;
                }
            } else if (Math.abs(targetTurretPos - turretPosCurrentTicks) > turretCloseToZero) {  // If not close to destination, temporarily keep turret where it is until lift raises.
                Robot.turretMotor.setTargetPosition(turretPosCurrentTicks);
                Robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Robot.turretMotor.setPower(turretSpeed);
                prevTargetTurretPos = turretPosCurrentTicks;
            }

            //----------------------------------------------------------//
            //LIFT MOTOR

            if (liftPosGroundButton) {
                targetLiftPos = groundLiftPosTicks;
            }

            if (liftPosLowButton) {
                targetLiftPos = lowLiftPosTicks;
                if (grabberServoCurrentPos == grabberServoOpenPos) {
                    grabberTrigger = 1;
                }
            }

            if (liftPosMediumButton) {
                targetLiftPos = mediumLiftPosTicks;
                if (grabberServoCurrentPos == grabberServoOpenPos) {
                    grabberTrigger = 1;
                }
            }
            if (liftPosHighButton) {
                targetLiftPos = highLiftPosTicks;
                if (grabberServoCurrentPos == grabberServoOpenPos) {
                    grabberTrigger = 1;
                }
            }

            // Allow lift to move when:  1) going up  2) Near the zero position  3) It won't go too low for turning
            if (liftPosCurrentTicks < targetLiftPos || Math.abs(turretPosCurrentTicks) < turretCloseToZero || targetLiftPos > liftMinHeightForTurning) {
                if (targetLiftPos != prevTargetLiftPos) {
                    Robot.liftMotor.setTargetPosition(targetLiftPos);
                    Robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Robot.liftMotor.setPower(liftSpeed * slowfactor);
                    prevTargetLiftPos = targetLiftPos;
                }
            } else {  // keep lift where is is
                Robot.liftMotor.setTargetPosition(liftPosCurrentTicks);
                Robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Robot.liftMotor.setPower(liftSpeed * slowfactor);
                prevTargetLiftPos = liftPosCurrentTicks;
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

            sinAngleRadians = Math.sin(stickAngleRadians);
            cosAngleRadians = Math.cos(stickAngleRadians);
            factor = 1 / Math.max(Math.abs(sinAngleRadians), Math.abs(cosAngleRadians));

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


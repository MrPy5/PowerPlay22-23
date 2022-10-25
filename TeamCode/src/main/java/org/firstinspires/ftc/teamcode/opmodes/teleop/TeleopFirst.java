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

        double slowModeSpeed = 1;
        double slowModeSlow = .3;
        double slowModeFast = 1;
        boolean slowMode = false;
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
        double turretCurrentDegrees = 0;
        double turretCloseToZero = 70;

        double turretTicksPerRevolution = 2786;
        double turretTicksPerDegree = turretTicksPerRevolution / 360;

        /**
        double forwardTurretPosTicks = 0;
        double rightTurretPosTicks = 696;
        double leftTurretPosTicks = -696;
        double backTurretPosTicks = 1393;
         **/

        double turretForwardDegrees = 0; //all rotation variables in degrees
        double turretRightDegrees = 90;
        double turretLeftDegrees = -90;
        double turretBackDegrees = 180;

        double turretTargetDegrees = 0;
        double turretPrevTargetDegrees = 0;


        //---------------------------------------------------------------//
        //LIFT VARIABLES

        double liftSpeedUp = 1;
        double liftSpeedDown = .5;
        double liftSpeedPower;

        double liftMotorTicksPerRevolution = 537;
        double liftSpoolDiameter = 7/8; //inches - if you have the correct spool diameter, everything else should just work
        double liftCascadeMultiplier = 3; // 3 stages of cascade stringing
        double liftTicksPerInch = liftMotorTicksPerRevolution / (liftSpoolDiameter * Math.PI * liftCascadeMultiplier);

        double liftCurrentHeight = 0; //all height variables are in inches
        double liftPickupHeight = 0;
        double liftJunctionGroundHeight = 1;
        double liftJunctionLowHeight = 15;
        double liftJunctionMediumHeight = 24;
        double liftJunctionHighHeight = 35;
        double liftMinHeightForTurning = 3;

        double liftHeightTarget = 0;
        double liftHeightPrevTarget = 0;


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
            liftCurrentHeight = Robot.liftMotor.getCurrentPosition() / liftTicksPerInch;
            turretCurrentDegrees = Robot.turretMotor.getCurrentPosition() / turretTicksPerDegree;


            //-----------------------------------------------------------//
            //SLO-MO CODE

            if (slowmoTrigger > triggerSensitivity) {
                if (slowmoTriggerReleased) {
                    if (slowMode == false) {
                        slowModeSpeed = slowModeSlow;
                        slowMode = true;
                    } else {
                        slowModeSpeed = slowModeFast;
                        slowMode = false;
                    }
                    slowmoTriggerReleased = false;
                }
            } else {
                slowmoTriggerReleased = true;
            }


            //---------------------------------------------------------------//
            //TURRET CODE

            if (turretPosForwardButton) {
                turretTargetDegrees = turretForwardDegrees;
            }
            if (turretPosLeftButton) {
                turretTargetDegrees = turretLeftDegrees;
            }
            if (turretPosRightButton) {
                turretTargetDegrees = turretRightDegrees;
            }
            if (turretPosBackButton) {
                turretTargetDegrees = turretBackDegrees;
            }

            if (liftCurrentHeight > liftMinHeightForTurning) {
                if (turretTargetDegrees != turretPrevTargetDegrees) {
                    Robot.turretMotor.setTargetPosition((int) (turretTargetDegrees * turretTicksPerDegree));
                    Robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Robot.turretMotor.setPower(turretSpeed);
                    turretPrevTargetDegrees = turretTargetDegrees;
                }
            } else if (Math.abs(turretTargetDegrees - turretCurrentDegrees) > turretCloseToZero) {  // If not close to destination, temporarily keep turret where it is until lift raises.
                Robot.turretMotor.setTargetPosition((int) (turretCurrentDegrees * turretTicksPerDegree));
                Robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Robot.turretMotor.setPower(turretSpeed);
                turretPrevTargetDegrees = turretCurrentDegrees;
            }

            //----------------------------------------------------------//
            //LIFT MOTOR

            if (liftPosGroundButton) {   // We need another button for lifeJunctionGroundHeight for scoring on a ground junction
                liftHeightTarget = liftPickupHeight;
            }

            if (liftPosLowButton) {
                liftHeightTarget = liftJunctionLowHeight;
                if (grabberServoCurrentPos == grabberServoOpenPos) {
                    grabberTrigger = 1;
                }
            }

            if (liftPosMediumButton) {
                liftHeightTarget = liftJunctionMediumHeight;
                if (grabberServoCurrentPos == grabberServoOpenPos) {
                    grabberTrigger = 1;
                }
            }
            if (liftPosHighButton) {
                liftHeightTarget = liftJunctionHighHeight;
                if (grabberServoCurrentPos == grabberServoOpenPos) {
                    grabberTrigger = 1;
                }
            }

            // Allow lift to move when:  1) going up  2) Turret near the zero position  3) It won't go too low for Turret turning
            if (liftCurrentHeight < liftHeightTarget || Math.abs(turretCurrentDegrees) < turretCloseToZero || liftHeightTarget > liftMinHeightForTurning) {
                if (liftHeightTarget != liftHeightPrevTarget) {
                    Robot.liftMotor.setTargetPosition((int)(liftHeightTarget * liftTicksPerInch));
                    Robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if (liftCurrentHeight < liftHeightTarget) {
                        liftSpeedPower = liftSpeedUp;
                    } else {
                        liftSpeedPower = liftSpeedDown;
                    }
                    Robot.liftMotor.setPower(liftSpeedPower);
                    liftHeightPrevTarget = liftHeightTarget;
                }
            } else {  // keep lift where is is
                Robot.liftMotor.setTargetPosition((int)(liftCurrentHeight * liftTicksPerInch));
                Robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Robot.liftMotor.setPower(liftSpeedUp);
                liftHeightPrevTarget = liftCurrentHeight;
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
                wheelPower = (.8 * wheelPower + .2) * slowModeSpeed;
            }

            stickAngleRadians = Math.atan2(leftStickY, leftStickX);

            stickAngleRadians = stickAngleRadians - Math.PI / 4; //adjust by 45 degrees

            sinAngleRadians = Math.sin(stickAngleRadians);
            cosAngleRadians = Math.cos(stickAngleRadians);
            factor = 1 / Math.max(Math.abs(sinAngleRadians), Math.abs(cosAngleRadians));

            rightX = rightStickX * slowModeSpeed * .8;

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
            telemetry.addData("Current Turret Position", turretCurrentDegrees);
            telemetry.addData("Target Turret Position", turretTargetDegrees);
            telemetry.addData("Current Lift Position", liftCurrentHeight);
            telemetry.addData("Target Lift Position", liftHeightTarget);

            telemetry.update();

        }
    }
}


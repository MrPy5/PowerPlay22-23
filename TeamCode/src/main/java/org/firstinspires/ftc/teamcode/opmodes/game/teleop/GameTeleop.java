//Checklist
/*

-180 turn for left
-


 */


package org.firstinspires.ftc.teamcode.opmodes.game.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.robot.Robot;


@TeleOp(name = "GAME TELEOP")

public class GameTeleop extends LinearOpMode {


    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap);
        waitForStart();


        double triggerSensitivity = 0.01;
        //---------------------------------------------------------------//
        //Mode
        boolean manualMode = false;
        boolean manualModeReleased = true;

        //---------------------------------------------------------------//
        //SLOW VARIABLES

        double slowModeSpeed = .8;
        double slowModeSlow = .4;
        double slowModeFast = .8;
        double slowModeGroundJuctionSlow = .2;

        double slowModeTurnSpeed = 0.6;
        double slowModeTurnSlow = 0.5;
        double slowModeTurnFast = 0.6;

        boolean slowMode = false;
        boolean slowmoTriggerReleased = true;


        //---------------------------------------------------------------//
        //GRABBER SERVO VARIABLES

        double grabberServoClosedPos = Robot.grabberServoClosedPos;
        double grabberServoOpenPos = Robot.grabberServoOpenPos;
        double grabberServoCurrentPos = 0.22;
        boolean grabberTriggerReleased = true;


        //---------------------------------------------------------------//
        //TURRET VARIABLES

        double turretSpeed = 0.5;
        double turretCurrentDegrees;
        double turretButtonChoiceTargetDegrees = 0;
        double turretCloseToZero = 70;

        double turretTicksPerDegree = Robot.turretTicksPerDegree;

        double turretForwardDegrees = 0; //all rotation variables in degrees
        double turretRightDegrees = 90;
        double turretLeftDegrees = -90;
        double turretBackDegrees = 180;

        double turretTargetDegrees = 0;
        double turretPrevTargetDegrees = 0;


        //---------------------------------------------------------------//
        //LIFT VARIABLES

        double liftSpeedUp = 1;
        double liftSpeedDown = .7;
        double liftSpeedPower;

        double liftTicksPerInch = Robot.liftTicksPerInch;

        double liftCurrentHeight; //all height variables are in inches
        double liftPickupHeight = 0;
        double liftJunctionGroundHeight = 2;
        double liftJunctionLowHeight = 14;
        double liftJunctionMediumHeight = 24;
        double liftJunctionHighHeight = 33;
        double liftMinHeightForTurning = 6;
        double liftMaximumHeight = 34;

        double liftHeightTarget = 0;
        double liftHeightPrevTarget = 0;

        double manualLiftIncrement = 2.5;



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

        double deadStickZone = 0.01;
        double wheelPowerMinToMove = 0.05;


        double ticksPerInch = Robot.ticksPerInch;

        //---------------------------------------------------------//
        //INIT SERVOS
        Robot.grabberServo.setPosition(grabberServoOpenPos);

        waitForStart();

        /*Robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Robot.liftMotor.setPower(0.03);
        sleep(800);
        Robot.liftMotor.setPower(0);
        Robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

        while (opModeIsActive()) {

            //---------------------------------------------------------------//
            //GAMEPAD2 CONTROLS = Lift + turret + grabbing

            double grabberTrigger = gamepad2.right_trigger;

            boolean turretPosForwardButton = gamepad2.dpad_up;
            boolean turretPosBackButton = gamepad2.dpad_down;
            boolean turretPosLeftButton = gamepad2.dpad_left;
            boolean turretPosRightButton = gamepad2.dpad_right;

            boolean liftPosGroundButton = gamepad2.right_bumper;
            boolean liftPosLowButton = gamepad2.a;
            boolean liftPosMediumButton = gamepad2.b;
            boolean liftPosGroundJunctionButton = gamepad2.x;
            boolean liftPosHighButton = gamepad2.y;

            boolean manualModeButton = gamepad2.start;


            //----------------------------------------------------------------//
            //GAMEPAD1 CONTROLS = Driving + slow mode + manual lift + score button

            double slowmoTrigger = gamepad1.left_trigger;

            double leftStickY = gamepad1.left_stick_y * -1 * slowModeSpeed;
            double leftStickX = gamepad1.left_stick_x * -1 * slowModeSpeed;
            double rightStickX = gamepad1.right_stick_x * slowModeTurnSpeed * .8;

            boolean liftPosUpManualButton = gamepad1.right_bumper;
            boolean liftPosDownManualButton = gamepad1.left_bumper;


            //---------------------------------------------------------------//
            //READ HARDWARE VALUES

            liftCurrentHeight = Robot.liftMotor.getCurrentPosition() / liftTicksPerInch;
            turretCurrentDegrees = Robot.turretMotor.getCurrentPosition() / turretTicksPerDegree;


            //-----------------------------------------------------------//
            //SLO-MO CODE

            if (slowmoTrigger > triggerSensitivity) {
                if (slowmoTriggerReleased) {
                    if (!slowMode) {
                        if (liftJunctionGroundHeight == liftHeightPrevTarget) {
                            slowModeSpeed = slowModeGroundJuctionSlow;
                            slowModeTurnSpeed = slowModeTurnSlow;
                            slowMode = true;
                        } else {
                            slowModeSpeed = slowModeSlow;
                            slowModeTurnSpeed = slowModeTurnSlow;
                            slowMode = true;
                        }

                    } else {
                        slowModeSpeed = slowModeFast;
                        slowModeTurnSpeed = slowModeTurnFast;
                        slowMode = false;
                    }
                    slowmoTriggerReleased = false;
                }
            } else {
                slowmoTriggerReleased = true;
            }

            //-----------------------------------------------------------//
            //Manual Mode Code

            if (manualModeButton) {
                if (manualModeReleased) {
                    if (!manualMode) {
                        manualMode = true;

                        Robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        Robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                        Robot.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        Robot.turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    } else {
                        manualMode = false;
                        Robot.turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        Robot.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        Robot.turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        Robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        Robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        Robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    }
                    manualModeReleased = false;
                }
            } else {
                manualModeReleased = true;
            }

            if (manualMode != true) {
                //----------------------------------------------------------//
                //LIFT MOTOR

                if (liftPosGroundButton) {
                    liftHeightTarget = liftPickupHeight;

                    turretButtonChoiceTargetDegrees = turretForwardDegrees;
                }
                if (liftPosGroundJunctionButton) {
                    liftHeightTarget = liftJunctionGroundHeight;
                }
                if (liftPosLowButton) {
                    liftHeightTarget = liftJunctionLowHeight;
                }
                if (liftPosMediumButton) {
                    liftHeightTarget = liftJunctionMediumHeight;
                }
                if (liftPosHighButton) {
                    liftHeightTarget = liftJunctionHighHeight;
                }

                //manual lift
                if (liftPosDownManualButton) {
                    if (liftCurrentHeight > manualLiftIncrement) {
                        liftHeightTarget = liftCurrentHeight - manualLiftIncrement;
                    }
                }
                if (liftPosUpManualButton) {
                    if (liftCurrentHeight < liftMaximumHeight - manualLiftIncrement) {
                        liftHeightTarget = liftCurrentHeight + manualLiftIncrement;
                    }
                }


                // Allow lift to move when:  1) going up  2) Turret near the zero position  3) It won't go too low for Turret turning
                if (liftCurrentHeight < liftHeightTarget || Math.abs(turretCurrentDegrees) < turretCloseToZero || liftHeightTarget > liftMinHeightForTurning) {
                    if (liftHeightTarget != liftHeightPrevTarget) {
                        Robot.liftMotor.setTargetPosition((int) (liftHeightTarget * liftTicksPerInch));
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
                    Robot.liftMotor.setTargetPosition((int) (liftCurrentHeight * liftTicksPerInch));
                    Robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Robot.liftMotor.setPower(liftSpeedUp);
                    liftHeightPrevTarget = liftCurrentHeight;
                }


                //---------------------------------------------------------------//
                //TURRET CODE

                if (turretPosForwardButton) {
                    turretButtonChoiceTargetDegrees = turretForwardDegrees;
                }
                if (turretPosLeftButton) {
                    turretButtonChoiceTargetDegrees = turretLeftDegrees;
                }
                if (turretPosRightButton) {
                    turretButtonChoiceTargetDegrees = turretRightDegrees;
                }

                if (turretPosBackButton) {
                    turretButtonChoiceTargetDegrees = turretBackDegrees;
                    if (turretCurrentDegrees < 0) {
                        turretButtonChoiceTargetDegrees = -turretButtonChoiceTargetDegrees;
                    }
                }


                if (liftCurrentHeight > liftMinHeightForTurning - 0.5) {
                    turretTargetDegrees = turretButtonChoiceTargetDegrees;
                } else {

                    if (Math.abs(turretTargetDegrees - turretCurrentDegrees) > turretCloseToZero) {  // If not close to destination, temporarily keep turret where it is until lift raises.
                        turretTargetDegrees = turretCurrentDegrees;
                    }
                }

                if (turretTargetDegrees != turretPrevTargetDegrees) {
                    Robot.turretMotor.setTargetPosition((int) (turretTargetDegrees * turretTicksPerDegree));
                    Robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Robot.turretMotor.setPower(turretSpeed);
                    turretPrevTargetDegrees = turretTargetDegrees;
                }

            }

            else {
                Robot.turretMotor.setPower(gamepad2.right_stick_x / 4);
                Robot.liftMotor.setPower(gamepad2.left_stick_y / 4);
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
                    Robot.grabberServo.setPosition(grabberServoCurrentPos);
                    grabberTriggerReleased = false;
                }
            } else {
                grabberTriggerReleased = true;
            }


            //Tightening code

            if (gamepad1.a) {

                Robot.liftMotor.setPower(0);
                Robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                Robot.liftMotor.setPower(0.05);
                sleep(500);
                Robot.liftMotor.setPower(0);
                Robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                Robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            }
            //---------------------------------------------------------//
            //DRIVING CODE

            wheelPower = Math.hypot(leftStickX, leftStickY);
            if (wheelPower > deadStickZone) {

                wheelPower = ((1 - wheelPowerMinToMove) * wheelPower + wheelPowerMinToMove);

            } else {
                wheelPower = 0;
            }


            stickAngleRadians = Math.atan2(leftStickY, leftStickX);

            stickAngleRadians = stickAngleRadians - Math.PI / 4; //adjust by 45 degrees

            sinAngleRadians = Math.sin(stickAngleRadians);
            cosAngleRadians = Math.cos(stickAngleRadians);
            factor = 1 / Math.max(Math.abs(sinAngleRadians), Math.abs(cosAngleRadians));


            lfPower = wheelPower * cosAngleRadians * factor + rightStickX;
            rfPower = wheelPower * sinAngleRadians * factor - rightStickX;
            lrPower = wheelPower * sinAngleRadians * factor + rightStickX;
            rrPower = wheelPower * cosAngleRadians * factor - rightStickX;

            /*if (Math.abs(lrPower + rrPower + lfPower + rfPower) < 0.01 && GetAverageVelocity(ticksPerInch) > breakingVelocity) {
                int multiplier = 1;
                if ((lrPower + rrPower + lfPower + rfPower) < 0.0) {
                    multiplier = -1;
                }
                lrPower = 0.01 * multiplier;
                rrPower = 0.01 * multiplier;
                lfPower = 0.01 * multiplier;
                rfPower = 0.01 * multiplier;
            }*/

            Robot.backLeft.setPower(lrPower);
            Robot.backRight.setPower(rrPower);
            Robot.frontLeft.setPower(lfPower);
            Robot.frontRight.setPower(rfPower);

            //---------------------------------------------------------------------//
            //TELEMETRY CODE
            telemetry.addData("LeftStickX:", gamepad1.left_stick_x);
            telemetry.addData("LeftStickY:", gamepad1.left_stick_y);
            telemetry.addData("WheelPower:", wheelPower);

            telemetry.addData("Turret Current Position (degrees):", turretCurrentDegrees);
            telemetry.addData("Turret Target Position (degrees):", turretTargetDegrees);
            telemetry.addData("Lift Current Position (inches):", liftCurrentHeight);
            telemetry.addData("Lift Target Position (inches):", liftHeightTarget);
            telemetry.addData("Average Velocity:", GetAverageVelocity(ticksPerInch));
            telemetry.update();

        }
    }

    public double GetAverageVelocity(double ticksPerInch) {
        double averageVelocity = 0;


        averageVelocity = (Robot.backRight.getVelocity() + Robot.backLeft.getVelocity() + Robot.frontLeft.getVelocity() + Robot.frontRight.getVelocity()) / 4;

        averageVelocity = (averageVelocity / ticksPerInch) / 12;
        return averageVelocity;
    }

}


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

        //---------------------------------------------------------------//
        //Manual Mode
        boolean autoScore = false;
        int scoreSteps = 0;


        //---------------------------------------------------------------//
        //Manual Mode
        boolean manualMode = false;
        boolean manualModeReleased = true;


        //---------------------------------------------------------------//
        //SLOW Mode
        boolean slowMode = false;
        boolean slowmoTriggerReleased = true;


        //---------------------------------------------------------------//
        //GRABBER SERVO VARIABLES
        double grabberServoCurrentPos = 0.22;
        boolean grabberTriggerReleased = true;


        //---------------------------------------------------------------//
        //TURRET VARIABLES
        double turretCurrentDegrees;
        double turretButtonChoiceTargetDegrees = 0;
        double turretTargetDegrees = 0;
        double turretPrevTargetDegrees = 0;


        //---------------------------------------------------------------//
        //LIFT VARIABLES
        double liftCurrentHeight;
        double liftSpeedPower;
        double liftHeightTarget = 0;
        double liftHeightPrevTarget = 0;
        double lastHeightTargetNoReset = 0;
        int timerToResetHeight = Robot.liftTimerForGuide;
        boolean startTiming = false;


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

        double avgWheelVelocityFPS;

        //---------------------------------------------------------//
        //INIT SERVOS
        Robot.grabberServo.setPosition(Robot.grabberServoOpenPos);


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

            double leftStickY = gamepad1.left_stick_y * -1 * Robot.slowModeSpeed;
            double leftStickX = gamepad1.left_stick_x * Robot.slowModeSpeed * 0.5;
            double rightStickX = gamepad1.right_stick_x * Robot.slowModeTurnSpeed * .8;

            boolean liftPosUpManualButton = gamepad1.right_bumper;
            boolean liftPosDownManualButton = gamepad1.left_bumper;

            double scoreButton = gamepad1.right_trigger;


            //---------------------------------------------------------------//
            //READ HARDWARE VALUES

            liftCurrentHeight = Robot.liftMotor.getCurrentPosition() / Robot.liftTicksPerInch;
            turretCurrentDegrees = Robot.turretMotor.getCurrentPosition() / Robot.turretTicksPerDegree;

            avgWheelVelocityFPS = GetAverageVelocity();

            //-----------------------------------------------------------//
            //SLO-MO CODE

            if (slowmoTrigger > Robot.triggerSensitivity) {
                if (slowmoTriggerReleased) {
                    if (!slowMode) {
                        if (Robot.liftJunctionGroundHeight == liftHeightPrevTarget) {
                            Robot.slowModeSpeed = Robot.slowModeGroundJuctionSlow;
                            Robot.slowModeTurnSpeed = Robot.slowModeTurnSlow;
                            slowMode = true;
                        } else {
                            Robot.slowModeSpeed = Robot.slowModeSlow;
                            Robot.slowModeTurnSpeed = Robot.slowModeTurnSlow;
                            slowMode = true;
                        }

                    } else {
                        Robot.slowModeSpeed = Robot.slowModeFast;
                        Robot.slowModeTurnSpeed = Robot.slowModeTurnFast;
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


            if (!manualMode) {

                //----------------------------------------------------------//
                //LIFT MOTOR

                if (liftPosGroundButton) {
                    liftHeightTarget = Robot.liftPickupHeight;

                    turretButtonChoiceTargetDegrees = Robot.turretForwardDegrees;
                    lastHeightTargetNoReset = Robot.liftPickupHeight;


                }
                if (liftPosGroundJunctionButton) {
                    liftHeightTarget = Robot.liftJunctionGroundHeight;
                    lastHeightTargetNoReset = Robot.liftJunctionGroundHeight;

                }
                if (liftPosLowButton) {
                    liftHeightTarget = Robot.liftJunctionLowHeight;
                    lastHeightTargetNoReset = Robot.liftJunctionLowHeight;
                }
                if (liftPosMediumButton) {
                    liftHeightTarget = Robot.liftJunctionMediumHeight;
                    lastHeightTargetNoReset = Robot.liftJunctionMediumHeight;
                }
                if (liftPosHighButton) {
                    liftHeightTarget = Robot.liftJunctionHighHeight;
                    lastHeightTargetNoReset = Robot.liftJunctionHighHeight;
                }

                //manual lift
                if (liftPosDownManualButton) {
                    if (liftCurrentHeight > Robot.manualLiftIncrement) {
                        liftHeightTarget = liftCurrentHeight - Robot.manualLiftIncrement;
                    }
                }
                if (liftPosUpManualButton) {
                    if (liftCurrentHeight < Robot.liftMaximumHeight - Robot.manualLiftIncrement) {
                        liftHeightTarget = liftCurrentHeight + Robot.manualLiftIncrement;
                    }
                }


                // Allow lift to move when:  1) going up  2) Turret near the zero position  3) It won't go too low for Turret turning
                if (liftCurrentHeight < liftHeightTarget || Math.abs(turretCurrentDegrees) < Robot.turretCloseToZero || liftHeightTarget > Robot.liftMinHeightForTurning) {
                    if (liftHeightTarget != liftHeightPrevTarget) {
                        Robot.liftMotor.setTargetPosition((int) (liftHeightTarget * Robot.liftTicksPerInch));
                        Robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        if (liftCurrentHeight < liftHeightTarget) {
                            liftSpeedPower = Robot.liftSpeedUp;
                        } else {
                            liftSpeedPower = Robot.liftSpeedDown;
                        }
                        Robot.liftMotor.setPower(liftSpeedPower);
                        liftHeightPrevTarget = liftHeightTarget;


                    }
                } else {  // keep lift where is is
                    Robot.liftMotor.setTargetPosition((int) (liftCurrentHeight * Robot.liftTicksPerInch));
                    Robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Robot.liftMotor.setPower(Robot.liftSpeedUp);
                    liftHeightPrevTarget = liftCurrentHeight;
                }

                //GUIDE SERVO CODE
                if (liftCurrentHeight > Robot.guideServoDeployHeight && liftHeightTarget != Robot.liftPickupHeight && grabberServoCurrentPos == Robot.grabberServoClosedPos && scoreSteps == 0) {
                    Robot.guideServo.setPosition(Robot.guideServoDown);
                    autoScore = true;
                }
                else {
                    Robot.guideServo.setPosition(Robot.guideServoUp);
                    autoScore = false;
                    if (scoreSteps == 1) {
                        scoreSteps = 2;
                    }

                }

                if (liftCurrentHeight <= (3.0/2.0) + (lastHeightTargetNoReset - 3) && scoreSteps == 2) {
                    Robot.grabberServo.setPosition(Robot.grabberServoOpenPos);
                }

                if (!Robot.liftMotor.isBusy() && scoreSteps == 2) {
                    liftHeightTarget = lastHeightTargetNoReset;
                    scoreSteps = 3;
                }

                else if (!Robot.liftMotor.isBusy() && scoreSteps == 3) {
                    liftHeightTarget = Robot.liftPickupHeight;
                    turretButtonChoiceTargetDegrees = Robot.turretForwardDegrees;
                    scoreSteps = 0;
                }





                //---------------------------------------------------------------//
                //TURRET CODE

                if (turretPosForwardButton) {
                    turretButtonChoiceTargetDegrees = Robot.turretForwardDegrees;
                }
                if (turretPosLeftButton) {
                    turretButtonChoiceTargetDegrees = Robot.turretLeftDegrees;
                }
                if (turretPosRightButton) {
                    turretButtonChoiceTargetDegrees = Robot.turretRightDegrees;
                }
                if (turretPosBackButton) {
                    turretButtonChoiceTargetDegrees = Robot.turretBackDegrees;
                    Robot.turretSpeed = 0.75;
                    if (turretCurrentDegrees < 0) {
                        turretButtonChoiceTargetDegrees = -turretButtonChoiceTargetDegrees;
                    }
                }


                if (liftCurrentHeight > Robot.liftMinHeightForTurning - 0.5) {
                    turretTargetDegrees = turretButtonChoiceTargetDegrees;
                } else {

                    if (Math.abs(turretTargetDegrees - turretCurrentDegrees) > Robot.turretCloseToZero) {  // If not close to destination, temporarily keep turret where it is until lift raises.
                        turretTargetDegrees = turretCurrentDegrees;
                    }
                }

                if (turretTargetDegrees != turretPrevTargetDegrees) {
                    Robot.turretMotor.setTargetPosition((int) (turretTargetDegrees * Robot.turretTicksPerDegree));
                    Robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Robot.turretMotor.setPower(Robot.turretSpeed);
                    turretPrevTargetDegrees = turretTargetDegrees;

                }

            }



            else {
                Robot.turretMotor.setPower(gamepad2.right_stick_x / 4);
                Robot.liftMotor.setPower(gamepad2.left_stick_y / 4);
            }




            //---------------------------------------------------------------//
            //GRABBER SERVO CODE

            if (grabberTrigger > Robot.triggerSensitivity) {
                if (grabberTriggerReleased  && !autoScore) {
                    if (grabberServoCurrentPos == Robot.grabberServoOpenPos) {
                        grabberServoCurrentPos = Robot.grabberServoClosedPos;
                    } else {
                        grabberServoCurrentPos = Robot.grabberServoOpenPos;

                    }
                    Robot.grabberServo.setPosition(grabberServoCurrentPos);
                    grabberTriggerReleased = false;
                }
            } else {
                grabberTriggerReleased = true;
            }


            if (autoScore && avgWheelVelocityFPS < 1) {
                if (Robot.colorSensorPole.green() > Robot.colorThreshold) {
                    scoreSteps = 1;
                    liftHeightTarget = lastHeightTargetNoReset - 3;
                }
                else {
                    grabberServoCurrentPos = Robot.grabberServoClosedPos;
                    Robot.grabberServo.setPosition(grabberServoCurrentPos);
                }

            }

            if (scoreButton > Robot.triggerSensitivity) {
                scoreSteps = 1;
                liftHeightTarget = lastHeightTargetNoReset - 3;
            }
            //---------------------------------------------------------//
            //Tightening Code

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
            if (wheelPower > Robot.deadStickZone) {

                wheelPower = ((1 - Robot.wheelPowerMinToMove) * wheelPower + Robot.wheelPowerMinToMove);

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
            telemetry.addData("Average Velocity:", GetAverageVelocity());
            telemetry.addData("Distance", Robot.colorSensorPole.green());
            telemetry.addData("Step", scoreSteps);
            telemetry.update();

        }
    }

    public double GetAverageVelocity() {
        double averageVelocity = 0;


        averageVelocity = (Robot.backRight.getVelocity() + Robot.backLeft.getVelocity() + Robot.frontLeft.getVelocity() + Robot.frontRight.getVelocity()) / 4;

        averageVelocity = (averageVelocity / Robot.ticksPerInch) / 12;
        return averageVelocity;
    }

}


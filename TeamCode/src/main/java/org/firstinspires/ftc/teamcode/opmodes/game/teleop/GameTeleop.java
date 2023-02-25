package org.firstinspires.ftc.teamcode.opmodes.game.teleop;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.robot.Robot;

import java.util.List;

@TeleOp(name = "GAME TELEOP")

public class GameTeleop extends LinearOpMode {


    @Override
    public void runOpMode() {

        //---Start Robot---//
        Robot robot = new Robot(hardwareMap, true);
        waitForStart();

        //---Bulk Reads---//
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        //---Manual Score Mode---//
        int scoreSteps = 0;
        int liftRaiseWaitTime = 200;

        ElapsedTime dropTimer = new ElapsedTime();


        //---Manual Mode---//
        boolean manualMode = false;
        boolean manualModeReleased = true;


        //---Auto Score---//
        boolean autoScoreMode = true;
        boolean autoScoreModeReleased = true;

        boolean allowAutoScore = true;


        //---Increment Lift---//
        boolean incrementUpReleased = true;
        boolean incrementDownReleased = true;


        //---Grabber---//
        double grabberServoCurrentPos;
        boolean grabberTriggerReleased = true;

        //---Cone Upright (cU)---//
        double cURightServoPos = Robot.cURightClosedPos;
        double cURightServoNextPos = cURightServoPos;
        double cULeftServoPos = Robot.cULeftClosedPos;
        double cULeftServoNextPos = cULeftServoPos;
        boolean cUExtendButtonReleased = true;
        boolean cURetractButtonReleased = true;
        double cULastMoveTime = 0;
        double cUIncrement = 0.01;

        ElapsedTime cUMoveTimer = new ElapsedTime();


        //---Turret---//
        double turretCurrentDegrees;
        double turretButtonChoiceTargetDegrees = 0;
        double turretTargetDegrees = 0;
        double turretPrevTargetDegrees = 0;


        //---Lift---//
        double liftCurrentHeight;
        double liftSpeedPower;
        double liftHeightTarget = 0;
        double liftHeightPrevTarget = 0;
        double lastHeightTargetNoReset = 0;

        //--Driving--//
        double sinAngleRadians;
        double cosAngleRadians;
        double factor;
        double wheelPower;
        double stickAngleRadians;
        double lfPower;
        double rfPower;
        double lrPower;
        double rrPower;

        double avgWheelVelocityFPS;

        //---Init Servos---//
        Robot.grabberServo.setPosition(Robot.grabberServoOpenPos);
        grabberServoCurrentPos = Robot.grabberServoOpenPos;

        Robot.coneUprightLeftServo.setPosition(Robot.cULeftClosedPos);
        Robot.coneUprightRightServo.setPosition(Robot.cURightClosedPos);

        Robot.guideServo.setPosition(Robot.guideServoUp);



        //---Start cU Move Timer---//
        cUMoveTimer.startTime();

        while (opModeIsActive()) {


            //---Gamepad2 controls---//
                // lift -> a,b,x,y
                // turret -> d-pad
                // manual mode -> start button
                // grab cone -> right trigger


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


            //---Gamepad1 controls---//
                // driving -> joysticks
                // lift increment -> bumpers
                // score button -> right trigger
                // auto score mode -> start


            double leftStickY = gamepad1.left_stick_y * -1;
            double leftStickX = gamepad1.left_stick_x *  0.8; // testing

            if (scoreSteps > 0) {
                leftStickX = 0;
                leftStickY = 0;
            }

            double rightStickX = gamepad1.right_stick_x * .8;

            boolean liftPosUpManualButton = gamepad1.right_bumper;
            boolean liftPosDownManualButton = gamepad1.left_bumper;

            double scoreButton = gamepad1.right_trigger;

            boolean autoScoreModeButton = gamepad1.start;

            boolean cUExtendButton = gamepad1.dpad_up;
            boolean cURetractButton = gamepad1.dpad_down;

            boolean resetToZero = gamepad1.touchpad_finger_1 && gamepad1.touchpad_finger_2;

            //---Read Hardware Values---//

            liftCurrentHeight = Robot.liftMotor.getCurrentPosition() / Robot.liftTicksPerInch;
            turretCurrentDegrees = Robot.turretMotor.getCurrentPosition() / Robot.turretTicksPerDegree;

            avgWheelVelocityFPS = GetAverageVelocity();

            int frontLeft = Robot.frontLeft.getCurrentPosition();
            int frontRight = Robot.frontRight.getCurrentPosition();
            int backLeft = Robot.backLeft.getCurrentPosition();
            int backRight = Robot.backRight.getCurrentPosition();

            //---Cone Uprighting---//


            if (!cUExtendButton) {
                cUExtendButtonReleased = true;
            }
            if (!cURetractButton) {
                cURetractButtonReleased = true;
            }

            if (liftHeightTarget >= Robot.liftJunctionLowHeight) {
                cULeftServoNextPos = Robot.cULeftClosedPos;
                cURightServoNextPos = Robot.cURightClosedPos;
            }

            if (cUExtendButton && cUExtendButtonReleased) {
                cUExtendButtonReleased = false;
                if (cULeftServoNextPos == Robot.cULeftClosedPos) {
                    cULeftServoNextPos = Robot.cULeftOpenPos;
                }
                else if (cULeftServoNextPos == Robot.cULeftOpenPos) {
                    cULeftServoNextPos = Robot.cULeftFlickPos;
                }
                if (cURightServoNextPos == Robot.cURightClosedPos) {
                    cURightServoNextPos = Robot.cURightOpenPos;
                }
                else if (cURightServoNextPos == Robot.cURightOpenPos) {
                    cURightServoNextPos = Robot.cURightFlickPos;
                }

            }

            if (cURetractButton && cURetractButtonReleased) {
                cURetractButtonReleased = false;
                if (cULeftServoNextPos == Robot.cULeftFlickPos) {
                    cULeftServoNextPos = Robot.cULeftOpenPos;
                }
                else if (cULeftServoNextPos == Robot.cULeftOpenPos) {
                    cULeftServoNextPos = Robot.cULeftClosedPos;
                }
                if (cURightServoNextPos == Robot.cURightFlickPos) {
                    cURightServoNextPos = Robot.cURightOpenPos;
                }
                else if (cURightServoNextPos == Robot.cURightOpenPos) {
                    cURightServoNextPos = Robot.cURightClosedPos;
                }
            }



            if (cULeftServoNextPos != cULeftServoPos || cURightServoNextPos != cURightServoPos) {
                if (liftCurrentHeight <= Robot.liftConeUprightHeight - .2) {
                    liftHeightTarget = Robot.liftConeUprightHeight;

                } else {
                    cURightServoPos = cURightServoNextPos;
                    cULeftServoPos = cULeftServoNextPos;

                    Robot.coneUprightLeftServo.setPosition(cULeftServoPos);
                    Robot.coneUprightRightServo.setPosition(cURightServoPos);

                    /*
                    if (cUMoveTimer.milliseconds() > cULastMoveTime + 25) {
                        cULastMoveTime = cUMoveTimer.milliseconds();

                        //Left
                        if (cULeftServoNextPos == Robot.cULeftFlickPos || cULeftServoPos == Robot.cULeftFlickPos) {
                            if (cULeftServoNextPos < cULeftServoPos) {
                                cULeftServoPos = cULeftServoPos - cUIncrement;

                                if (cULeftServoPos < cULeftServoNextPos) {
                                    cULeftServoPos = cULeftServoNextPos;
                                }

                            } else {
                                cULeftServoPos = cULeftServoPos + cUIncrement;

                                if (cULeftServoPos > cULeftServoNextPos) {
                                    cULeftServoPos = cULeftServoNextPos;
                                }

                            }
                        }
                        else {
                            cULeftServoPos = cULeftServoNextPos;
                        }

                        //Right
                        if (cURightServoNextPos == Robot.cURightFlickPos || cURightServoPos == Robot.cURightFlickPos) {
                            if (cURightServoNextPos < cURightServoPos) {
                                cURightServoPos = cURightServoPos - cUIncrement;

                                if (cURightServoPos < cURightServoNextPos) {
                                    cURightServoPos = cURightServoNextPos;
                                }

                            } else {
                                cURightServoPos = cURightServoPos + cUIncrement;

                                if (cURightServoPos > cURightServoNextPos) {
                                    cURightServoPos = cURightServoNextPos;
                                }

                            }
                        }
                        else {
                            cURightServoPos = cURightServoNextPos;
                        }


                        Robot.coneUprightLeftServo.setPosition(cULeftServoPos);
                        Robot.coneUprightRightServo.setPosition(cURightServoPos);
                    }

                     */
                }
            }

            //---Reset Robot lift and turret---//

            if (resetToZero) {
                resetTurretAndLift();
            }


            //---Manual Mode Button Code---//

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

            //---Auto Score Mode Button Code---//
            if (autoScoreModeButton) {
                if (autoScoreModeReleased) {
                    autoScoreMode = !autoScoreMode;
                    autoScoreModeReleased = false;
                }
            }
            else {
                autoScoreModeReleased = true;
            }

            //---Lift and Turret---//

            if (!manualMode) {


                //---Lift---//

                if (liftPosGroundButton) {
                    liftHeightTarget = Robot.liftPickupHeight;
                    lastHeightTargetNoReset = Robot.liftPickupHeight;

                    turretButtonChoiceTargetDegrees = Robot.turretForwardDegrees;

                }
                if (liftPosGroundJunctionButton) {
                    liftHeightTarget = Robot.liftJunctionGroundHeight;
                    lastHeightTargetNoReset = Robot.liftJunctionGroundHeight;
                    scoreSteps = 0; // Panic button for incase auto score cant finish


                }
                if (liftPosLowButton) {
                    liftHeightTarget = Robot.liftJunctionLowHeight;
                    lastHeightTargetNoReset = Robot.liftJunctionLowHeight;
                    scoreSteps = 0; // Panic button for incase auto score cant finish

                }
                if (liftPosMediumButton) {
                    liftHeightTarget = Robot.liftJunctionMediumHeight;
                    lastHeightTargetNoReset = Robot.liftJunctionMediumHeight;
                    scoreSteps = 0; // Panic button for incase auto score cant finish

                }
                if (liftPosHighButton) {
                    liftHeightTarget = Robot.liftJunctionHighHeight;
                    lastHeightTargetNoReset = Robot.liftJunctionHighHeight;
                    scoreSteps = 0; // Panic button for incase auto score cant finish

                }

                // manual lift
                if (liftPosDownManualButton) {
                    if (liftCurrentHeight > Robot.manualLiftIncrement && incrementDownReleased) {
                        liftHeightTarget = liftCurrentHeight - Robot.manualLiftIncrement;
                        incrementDownReleased = false;
                    }
                }
                else {
                    incrementDownReleased = true;
                }
                if (liftPosUpManualButton) {
                    if (liftCurrentHeight < Robot.liftMaximumHeight - Robot.manualLiftIncrement && incrementUpReleased) {
                        liftHeightTarget = liftCurrentHeight + Robot.manualLiftIncrement;
                        incrementUpReleased = false;
                    }
                }
                else {
                    incrementUpReleased = true;
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


                //---Turret---//

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

            //---Manual Mode---//
            else {
                if (Math.abs(gamepad2.left_stick_y / 4) > 0.01) {
                    Robot.liftMotor.setPower((gamepad2.left_stick_y / 4) * -1);
                }
                else {
                    Robot.liftMotor.setPower(0);
                }
                Robot.turretMotor.setPower(gamepad2.right_stick_x / 4);
            }


            //---Grabber Servo---//

            if (grabberTrigger > Robot.triggerSensitivity) {
                if (grabberTriggerReleased) {
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


            //---Guide Servo---//

            if (liftCurrentHeight > Robot.guideServoDeployHeight && liftHeightTarget >= Robot.liftJunctionLowHeight && grabberServoCurrentPos == Robot.grabberServoClosedPos && scoreSteps == 0) {
                Robot.guideServo.setPosition(Robot.guideServoDown);

            }
            else {
                Robot.guideServo.setPosition(Robot.guideServoUp);
            }


            //---Auto Score---//

            if (liftCurrentHeight > Robot.guideServoDeployHeight && autoScoreMode) {
                if (allowAutoScore && scoreSteps == 0) {
                    scoreSteps = CheckForPole(avgWheelVelocityFPS, lastHeightTargetNoReset, frontLeft, frontRight, backLeft, backRight, GetAveragePosition());
                    if (scoreSteps != 0) {
                        allowAutoScore = false;
                    }
                }
            }
            if (liftCurrentHeight < Robot.liftConeUprightHeight) {
                allowAutoScore = true;
            }

            //---Manual Score---//
            if (scoreButton > Robot.triggerSensitivity && scoreSteps == 0) {
                scoreSteps = 1;
            }

            //---Score Steps---//

            if (!Robot.liftMotor.isBusy() && scoreSteps == 3) {
                sleep(250);

                liftHeightTarget = lastHeightTargetNoReset;
                scoreSteps = 0;

            }
            if (scoreSteps == 2 && dropTimer.milliseconds() > liftRaiseWaitTime) {
                grabberServoCurrentPos = Robot.grabberServoOpenPos;
                Robot.grabberServo.setPosition(grabberServoCurrentPos);
                scoreSteps = 3;
            }
            if (scoreSteps == 1) {
                Robot.guideServo.setPosition(Robot.guideServoUp);

                liftHeightTarget = lastHeightTargetNoReset - 5;
                scoreSteps = 2;

                dropTimer.reset();
            }


            //---Tightening Code---//

            if (gamepad1.a && liftCurrentHeight < 2) {

                Robot.liftMotor.setPower(0);
                Robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                Robot.liftMotor.setPower(0.05);
                sleep(500);
                Robot.liftMotor.setPower(0);
                Robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                Robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            //---Driving Code---//

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

            if (liftCurrentHeight > Robot.liftJunctionLowHeight && Math.abs(avgWheelVelocityFPS) > 0.3) {
                if (leftStickX == 0 && leftStickY == 0) {
                    if (avgWheelVelocityFPS > 0) {
                        lfPower = 0.01;
                        rfPower = 0.01;
                        lrPower = 0.01;
                        rrPower = 0.01;
                    } else {
                        lfPower = -0.01;
                        rfPower = -0.01;
                        lrPower = -0.01;
                        rrPower = -0.01;
                    }
                }
            }

            Robot.backLeft.setPower(lrPower);
            Robot.backRight.setPower(rrPower);
            Robot.frontLeft.setPower(lfPower);
            Robot.frontRight.setPower(rfPower);

            //---Telemetry---//

            if (manualMode) {
                telemetry.addData("---MANUAL MODE---", "");
            }
            if (autoScoreMode) {
                telemetry.addData("---Auto Score Mode---", "");
            }
            telemetry.addData("LeftStickX:", gamepad1.left_stick_x);
            telemetry.addData("LeftStickY:", gamepad1.left_stick_y);

            telemetry.addData("WheelPower:", wheelPower);

            telemetry.addData("Lift Height:", liftCurrentHeight);

            /*telemetry.addData("Turret Current Position (degrees):", turretCurrentDegrees);
            telemetry.addData("Turret Target Position (degrees):", turretTargetDegrees);
            telemetry.addData("Lift Current Position (inches):", liftCurrentHeight);
            telemetry.addData("Lift Target Position (inches):", liftHeightTarget);*/
            telemetry.addData("Average Velocity:", GetAverageVelocity());
            telemetry.addData("Distance", Robot.colorSensorPole.green());
            telemetry.addData("Steps", scoreSteps);

            telemetry.update();

            boolean colorOverThreshold = Robot.colorSensorPole.green() > Robot.colorThreshold;
            String color = String.valueOf(colorOverThreshold);
            Log.d("ColorDetection", "Green:" + color);

        }
    }

    public double GetAverageVelocity() {
        double averageVelocity;
        averageVelocity = (Robot.backRight.getVelocity() + Robot.backLeft.getVelocity() + Robot.frontLeft.getVelocity() + Robot.frontRight.getVelocity()) / 4;
        averageVelocity = (averageVelocity / Robot.ticksPerInch) / 12;
        return averageVelocity;
    }
    public double GetAveragePosition() {
        return (Robot.backRight.getCurrentPosition() + Robot.backLeft.getCurrentPosition() + Robot.frontLeft.getCurrentPosition() + Robot.frontRight.getCurrentPosition()) / 4;
    }


    /*public double CheckForPole(boolean autoScore, double avgWheelVelocityFPS, double lastHeightTargetNoReset, double grabberServoCurrentPos, int frontLeft, int frontRight, int backLeft, int backRight) {


        if (autoScore && Math.abs(avgWheelVelocityFPS) < 1.5) {
            if (Robot.colorSensorPole.green() > Robot.colorThreshold) {

                Robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                Robot.frontLeft.setTargetPosition(frontLeft);
                Robot.frontRight.setTargetPosition(frontRight);
                Robot.backLeft.setTargetPosition(backLeft);
                Robot.backRight.setTargetPosition(backRight);

                Robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                Robot.frontLeft.setPower(0.8);
                Robot.frontRight.setPower(0.8);
                Robot.backLeft.setPower(0.8);
                Robot.backRight.setPower(0.8);

                Robot.guideServo.setPosition(Robot.guideServoUp);

                while (Robot.frontLeft.isBusy() || Robot.frontRight.isBusy() || Robot.backLeft.isBusy() || Robot.backRight.isBusy()) {

                    Robot.liftMotor.setTargetPosition((int) ((lastHeightTargetNoReset) * Robot.liftTicksPerInch));
                    Robot.liftMotor.setPower(0.8);
                }

                int counter = 200;
                while (counter != 0) {
                    counter = counter - 1;
                }

                Robot.liftMotor.setTargetPosition((int) ((lastHeightTargetNoReset - 5) * Robot.liftTicksPerInch));
                Robot.liftMotor.setPower(0.8);

                while (true) {


                    if (!Robot.liftMotor.isBusy()) {
                        grabberServoCurrentPos = Robot.grabberServoOpenPos;
                        Robot.grabberServo.setPosition(grabberServoCurrentPos);
                        break;
                    }
                }

                Robot.liftMotor.setTargetPosition((int) ((lastHeightTargetNoReset) * Robot.liftTicksPerInch));
                while (true) {

                    Robot.liftMotor.setPower(Robot.liftSpeedUp);
                    if (!Robot.liftMotor.isBusy()) {
                        break;
                    }
                }
            }

        }

        Robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        return grabberServoCurrentPos;
    } */

    public int CheckForPole(double avgWheelVelocityFPS, double lastHeightTargetNoReset, int frontLeft, int frontRight, int backLeft, int backRight, double originalAveragePosition) {

        //int adjustmentAmount = (int) (avgWheelVelocityFPS * Robot.ticksPerInch * 0);

        if (Math.abs(avgWheelVelocityFPS) < 1.2) {
            if (Robot.colorSensorPole.green() > Robot.colorThreshold && (Robot.liftMotor.getCurrentPosition() * Robot.ticksPerInch) >= lastHeightTargetNoReset - 0.5) {
                double speed = 0.01;

                Robot.frontLeft.setPower(speed);
                Robot.frontRight.setPower(speed);
                Robot.backLeft.setPower(speed);
                Robot.backRight.setPower(speed);

                // - adjustmentAmount

                Robot.frontLeft.setTargetPosition(frontLeft);
                Robot.frontRight.setTargetPosition(frontRight);
                Robot.backLeft.setTargetPosition(backLeft);
                Robot.backRight.setTargetPosition(backRight);


                Robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                /**
                Robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                **/




                while (opModeIsActive() && Robot.frontLeft.isBusy()){ //(Math.abs(originalAveragePosition - adjustmentAmount) - Math.abs(GetAveragePosition())) * Robot.ticksPerInch > 0.25) { //loop until within quarter inch of target
                    telemetry.addData("distance travelled", (Math.abs(originalAveragePosition) - Math.abs(GetAveragePosition())) * Robot.ticksPerInch);
                    telemetry.update();

                    if (speed < 0.1 && Math.abs(GetAverageVelocity()) < 0.2) {
                        speed += 0.01;
                    }
                    Robot.frontLeft.setPower(speed);
                    Robot.frontRight.setPower(speed);
                    Robot.backLeft.setPower(speed);
                    Robot.backRight.setPower(speed);
                }


                Robot.frontLeft.setPower(0.01);
                Robot.frontRight.setPower(0.01);
                Robot.backLeft.setPower(0.01);
                Robot.backRight.setPower(0.01);

                Robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


                telemetry.addLine("Finished travel back");
                telemetry.update();


                ElapsedTime timeHeldFor = new ElapsedTime();
                ElapsedTime totalTime = new ElapsedTime();
                totalTime.startTime();
                timeHeldFor.startTime();

                while (timeHeldFor.milliseconds() < 200 && opModeIsActive()) {
                    if (Robot.colorSensorPole.green() < Robot.colorThreshold) {
                            timeHeldFor.reset();
                    }
                    if (gamepad1.right_trigger > Robot.triggerSensitivity) {
                        return 1;
                    }
                    if (gamepad1.left_trigger > Robot.triggerSensitivity) {
                        return 0;
                    }
                    telemetry.addLine("In loop");
                    telemetry.update();
                }

                return 1;
            }
        }
        return 0;
    }

    public void resetTurretAndLift() {

        //---Readjust to 0---//
        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        Robot.liftMotor.setTargetPosition((int) (7 * Robot.liftTicksPerInch));
        Robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Robot.liftMotor.setPower(1);

        while (Robot.liftMotor.isBusy() && opModeIsActive() && gamepad1.a == false) {}


        Robot.turretMotor.setTargetPosition(0);
        Robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Robot.turretMotor.setPower(0.8);


        while (Robot.turretMotor.isBusy() && opModeIsActive() && gamepad1.a == false) {}

        Robot.liftMotor.setTargetPosition(0);
        Robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Robot.liftMotor.setPower(0.5);
        Robot.grabberServo.setPosition(Robot.grabberServoOpenPos);
    }

}


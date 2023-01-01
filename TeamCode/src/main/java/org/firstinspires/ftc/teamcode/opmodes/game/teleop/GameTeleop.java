//Checklist
/*

-180 turn for left
-


 */


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

        Robot robot = new Robot(hardwareMap, true);
        waitForStart();

        /*List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }*/

        //---------------------------------------------------------------//
        //Manual Mode
        boolean autoScore = false;
        int scoreSteps = 0;
        int liftRaiseWaitTime = 150;

        ElapsedTime dropTimer = new ElapsedTime();


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
        double grabberServoCurrentPos;
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

        //---------------------------------------------------------//
        //DRIVING CODE VARIABLES
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

        //---------------------------------------------------------//
        //INIT SERVOS
        robot.grabberServo.setPosition(robot.grabberServoOpenPos);
        grabberServoCurrentPos = robot.grabberServoOpenPos;

        while (opModeIsActive()) {
            //Log.d("LOOPED", "logged");
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

            boolean uprightCone = gamepad2.left_bumper;


            //----------------------------------------------------------------//
            //GAMEPAD1 CONTROLS = Driving + slow mode + manual lift + score button

            double slowmoTrigger = gamepad1.left_trigger;

            double leftStickY = gamepad1.left_stick_y * -1 * robot.slowModeSpeed;
            double leftStickX = gamepad1.left_stick_x * robot.slowModeSpeed * 0.8; // testing
            double rightStickX = gamepad1.right_stick_x * robot.slowModeTurnSpeed * .8;

            boolean liftPosUpManualButton = gamepad1.right_bumper;
            boolean liftPosDownManualButton = gamepad1.left_bumper;

            double scoreButton = gamepad1.right_trigger;



            //---------------------------------------------------------------//
            //READ HARDWARE VALUES

            liftCurrentHeight = robot.liftMotor.getCurrentPosition() / robot.liftTicksPerInch;
            turretCurrentDegrees = robot.turretMotor.getCurrentPosition() / robot.turretTicksPerDegree;

            avgWheelVelocityFPS = GetAverageVelocity(robot);


            //-----------------------------------------------------------//
            //SLO-MO CODE

            if (slowmoTrigger > robot.triggerSensitivity) {
                if (slowmoTriggerReleased) {
                    if (!slowMode) {
                        if (robot.liftJunctionGroundHeight == liftHeightPrevTarget) {
                            robot.slowModeSpeed = robot.slowModeGroundJuctionSlow;
                        } else {
                            robot.slowModeSpeed = robot.slowModeSlow;
                        }
                        robot.slowModeTurnSpeed = robot.slowModeTurnSlow;
                        slowMode = true;

                    } else {
                        robot.slowModeSpeed = robot.slowModeFast;
                        robot.slowModeTurnSpeed = robot.slowModeTurnFast;
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

                        robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                        robot.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    } else {
                        manualMode = false;
                        robot.turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    }
                    manualModeReleased = false;
                }
            } else {
                manualModeReleased = true;
            }



            if (!manualMode) {

                //----------------------------------------------------------//
                //LIFT MOTOR
                if (liftCurrentHeight > robot.guideServoDeployHeight) {
                    Log.d("POLECHECK", "AT THE POLE CHECK");
                    grabberServoCurrentPos = CheckForPole(robot, avgWheelVelocityFPS, robot.liftJunctionHighHeight, grabberServoCurrentPos, robot.frontLeft.getCurrentPosition(), robot.frontRight.getCurrentPosition(), robot.backLeft.getCurrentPosition(), robot.backRight.getCurrentPosition());

                }

                if (liftPosGroundButton) {
                    liftHeightTarget = robot.liftPickupHeight;
                    lastHeightTargetNoReset = robot.liftPickupHeight;

                    turretButtonChoiceTargetDegrees = robot.turretForwardDegrees;

                }
                if (liftPosGroundJunctionButton) {
                    liftHeightTarget = robot.liftJunctionGroundHeight;
                    lastHeightTargetNoReset = robot.liftJunctionGroundHeight;

                }
                if (liftPosLowButton) {
                    liftHeightTarget = robot.liftJunctionLowHeight;
                    lastHeightTargetNoReset = robot.liftJunctionLowHeight;
                }
                if (liftPosMediumButton) {
                    liftHeightTarget = robot.liftJunctionMediumHeight;
                    lastHeightTargetNoReset = robot.liftJunctionMediumHeight;
                }
                if (liftPosHighButton) {
                    liftHeightTarget = robot.liftJunctionHighHeight;
                    lastHeightTargetNoReset = robot.liftJunctionHighHeight;
                }

                // manual lift
                if (liftPosDownManualButton) {
                    if (liftCurrentHeight > robot.manualLiftIncrement) {
                        liftHeightTarget = liftCurrentHeight - robot.manualLiftIncrement;
                    }
                }
                if (liftPosUpManualButton) {
                    if (liftCurrentHeight < robot.liftMaximumHeight - robot.manualLiftIncrement) {
                        liftHeightTarget = liftCurrentHeight + robot.manualLiftIncrement;
                    }
                }


                // Allow lift to move when:  1) going up  2) Turret near the zero position  3) It won't go too low for Turret turning
                if (liftCurrentHeight < liftHeightTarget || Math.abs(turretCurrentDegrees) < robot.turretCloseToZero || liftHeightTarget > robot.liftMinHeightForTurning) {
                    if (liftHeightTarget != liftHeightPrevTarget) {
                        robot.liftMotor.setTargetPosition((int) (liftHeightTarget * robot.liftTicksPerInch));
                        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        if (liftCurrentHeight < liftHeightTarget) {
                            liftSpeedPower = robot.liftSpeedUp;
                        } else {
                            liftSpeedPower = robot.liftSpeedDown;
                        }
                        robot.liftMotor.setPower(liftSpeedPower);
                        liftHeightPrevTarget = liftHeightTarget;


                    }
                } else {  // keep lift where is is
                    robot.liftMotor.setTargetPosition((int) (liftCurrentHeight * robot.liftTicksPerInch));
                    robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotor.setPower(robot.liftSpeedUp);
                    liftHeightPrevTarget = liftCurrentHeight;
                }

                //GUIDE SERVO CODE
                if (liftCurrentHeight > robot.guideServoDeployHeight && liftHeightTarget >= robot.liftJunctionLowHeight && grabberServoCurrentPos == robot.grabberServoClosedPos && scoreSteps == 0) {
                    robot.guideServo.setPosition(robot.guideServoDown);

                }
                else {
                    robot.guideServo.setPosition(robot.guideServoUp);




                }


                //---------------------------------------------------------------//
                //TURRET CODE

                if (turretPosForwardButton) {
                    turretButtonChoiceTargetDegrees = robot.turretForwardDegrees;
                }
                if (turretPosLeftButton) {
                    turretButtonChoiceTargetDegrees = robot.turretLeftDegrees;
                }
                if (turretPosRightButton) {
                    turretButtonChoiceTargetDegrees = robot.turretRightDegrees;
                }
                if (turretPosBackButton) {
                    turretButtonChoiceTargetDegrees = robot.turretBackDegrees;
                    robot.turretSpeed = 0.75;
                    if (turretCurrentDegrees < 0) {
                        turretButtonChoiceTargetDegrees = -turretButtonChoiceTargetDegrees;
                    }
                }


                if (liftCurrentHeight > robot.liftMinHeightForTurning - 0.5) {
                    turretTargetDegrees = turretButtonChoiceTargetDegrees;
                } else {

                    if (Math.abs(turretTargetDegrees - turretCurrentDegrees) > robot.turretCloseToZero) {  // If not close to destination, temporarily keep turret where it is until lift raises.
                        turretTargetDegrees = turretCurrentDegrees;
                    }
                }

                if (turretTargetDegrees != turretPrevTargetDegrees) {
                    robot.turretMotor.setTargetPosition((int) (turretTargetDegrees * robot.turretTicksPerDegree));
                    robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.turretMotor.setPower(robot.turretSpeed);
                    turretPrevTargetDegrees = turretTargetDegrees;

                }

            }

            //MANUAL MODE
            else {
                robot.turretMotor.setPower(gamepad2.right_stick_x / 4);
                robot.liftMotor.setPower(gamepad2.left_stick_y / 4);
            }


            //---------------------------------------------------------------//
            //GRABBER SERVO CODE

            if (grabberTrigger > robot.triggerSensitivity) {
                if (grabberTriggerReleased  && !autoScore) {
                    if (grabberServoCurrentPos == robot.grabberServoOpenPos) {
                        grabberServoCurrentPos = robot.grabberServoClosedPos;
                    } else {
                        grabberServoCurrentPos = robot.grabberServoOpenPos;

                    }
                    robot.grabberServo.setPosition(grabberServoCurrentPos);
                    grabberTriggerReleased = false;
                }
            } else {
                grabberTriggerReleased = true;
            }

            if (uprightCone) {
                robot.grabberServo.setPosition(robot.grabberServoUprightPos);
                liftHeightTarget = robot.liftUprightHeight;
            }



            if (scoreButton > robot.triggerSensitivity) {
                scoreSteps = 1;
            }

            if (scoreSteps == 3 && dropTimer.milliseconds() > liftRaiseWaitTime) {
                liftHeightTarget = lastHeightTargetNoReset;
                scoreSteps = 0;
            }
            if (robot.liftMotor.getTargetPosition() != 0) {
                if (scoreSteps == 2 && robot.liftMotor.getCurrentPosition() / robot.liftMotor.getTargetPosition() > 0.98) {
                    grabberServoCurrentPos = robot.grabberServoOpenPos;
                    robot.grabberServo.setPosition(grabberServoCurrentPos);
                    dropTimer.reset();
                    scoreSteps = 3;

                }
            }
            else {
                if (scoreSteps == 2 && !robot.liftMotor.isBusy()) {
                    grabberServoCurrentPos = robot.grabberServoOpenPos;
                    robot.grabberServo.setPosition(grabberServoCurrentPos);
                    dropTimer.reset();
                    scoreSteps = 3;
                }
            }
            if (scoreSteps == 1) {
                robot.guideServo.setPosition(robot.guideServoUp);
                liftHeightTarget = lastHeightTargetNoReset - 5;
                scoreSteps = 2;
            }

            //---------------------------------------------------------//
            //Tightening Code

            if (gamepad1.a && liftCurrentHeight < 2) {

                robot.liftMotor.setPower(0);
                robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.liftMotor.setPower(0.05);
                sleep(500);
                robot.liftMotor.setPower(0);
                robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            //---------------------------------------------------------//
            //DRIVING CODE

            wheelPower = Math.hypot(leftStickX, leftStickY);
            if (wheelPower > robot.deadStickZone) {

                wheelPower = ((1 - robot.wheelPowerMinToMove) * wheelPower + robot.wheelPowerMinToMove);

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

            if (liftCurrentHeight > robot.liftJunctionLowHeight && Math.abs(avgWheelVelocityFPS) > 1.5) {
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


            robot.backLeft.setPower(lrPower);
            robot.backRight.setPower(rrPower);
            robot.frontLeft.setPower(lfPower);
            robot.frontRight.setPower(rfPower);

            //---------------------------------------------------------------------//
            //TELEMETRY CODE
            if (manualMode) {
                telemetry.addData("---MANUAL MODE---", "");
            }
            telemetry.addData("LeftStickX:", gamepad1.left_stick_x);
            telemetry.addData("LeftStickY:", gamepad1.left_stick_y);

            telemetry.addData("WheelPower:", wheelPower);

            /*telemetry.addData("Turret Current Position (degrees):", turretCurrentDegrees);
            telemetry.addData("Turret Target Position (degrees):", turretTargetDegrees);
            telemetry.addData("Lift Current Position (inches):", liftCurrentHeight);
            telemetry.addData("Lift Target Position (inches):", liftHeightTarget);*/
            telemetry.addData("Average Velocity:", GetAverageVelocity(robot));
            telemetry.addData("Distance", robot.colorSensorPole.green());
            telemetry.addData("Steps", scoreSteps);

            telemetry.update();

        }
    }

    public double GetAverageVelocity(Robot robot) {
        double averageVelocity;
        averageVelocity = (robot.backRight.getVelocity() + robot.backLeft.getVelocity() + robot.frontLeft.getVelocity() + robot.frontRight.getVelocity()) / 4;
        averageVelocity = (averageVelocity / robot.ticksPerInch) / 12;
        return averageVelocity;
    }


    public double CheckForPole(Robot robot, double avgWheelVelocityFPS, double lastHeightTargetNoReset, double grabberServoCurrentPos, int frontLeft, int frontRight, int backLeft, int backRight) {
        if (Math.abs(avgWheelVelocityFPS) < 1.5) {
            if (robot.colorSensorPole.green() > robot.colorThreshold) {
                robot.frontLeft.setTargetPosition(frontLeft - (int) (avgWheelVelocityFPS * robot.ticksPerInch));
                robot.frontRight.setTargetPosition(frontRight - (int) (avgWheelVelocityFPS * robot.ticksPerInch));
                robot.backLeft.setTargetPosition(backLeft - (int) (avgWheelVelocityFPS * robot.ticksPerInch));
                robot.backRight.setTargetPosition(backRight - (int) (avgWheelVelocityFPS * robot.ticksPerInch));

                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.frontLeft.setPower(0.4);
                robot.frontRight.setPower(0.4);
                robot.backLeft.setPower(0.4);
                robot.backRight.setPower(0.4);



                robot.liftMotor.setTargetPosition((int) ((lastHeightTargetNoReset) * robot.liftTicksPerInch));
                robot.liftMotor.setPower(0.8);
                while (opModeIsActive() && (robot.frontLeft.isBusy() || robot.frontRight.isBusy() || robot.backLeft.isBusy() || robot.backRight.isBusy() || robot.liftMotor.isBusy())) {}

                robot.liftMotor.setTargetPosition((int) ((lastHeightTargetNoReset - 5) * robot.liftTicksPerInch));
                robot.liftMotor.setPower(0.8);

                while (opModeIsActive() && robot.liftMotor.isBusy()) {
                    if ((float) (robot.liftMotor.getCurrentPosition() / robot.liftMotor.getTargetPosition()) > 0.9) {
                        robot.guideServo.setPosition(robot.guideServoUp);
                    }
                }

                grabberServoCurrentPos = robot.grabberServoOpenPos;
                robot.grabberServo.setPosition(grabberServoCurrentPos);

                robot.liftMotor.setTargetPosition((int) ((lastHeightTargetNoReset) * robot.liftTicksPerInch));
                robot.liftMotor.setPower(robot.liftSpeedUp);
                while (opModeIsActive() && robot.liftMotor.isBusy()) {}


                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);

                robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
        return grabberServoCurrentPos;
    }

}


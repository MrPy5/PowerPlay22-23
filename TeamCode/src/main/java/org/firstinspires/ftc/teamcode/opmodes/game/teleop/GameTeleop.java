package org.firstinspires.ftc.teamcode.opmodes.game.teleop;

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

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //---------------------------------------------------------------//
        //Manual Mode
        int scoreSteps = 0;
        int liftRaiseWaitTime = 150;

        ElapsedTime dropTimer = new ElapsedTime();


        //---------------------------------------------------------------//
        //Manual Mode
        boolean manualMode = false;
        boolean manualModeReleased = true;

        //---------------------------------------------------------------//
        //Auto Score
        boolean autoScoreMode = false;
        boolean autoScoreModeReleased = true;

        boolean allowAutoScore = true;

        //---------------------------------------------------------------//
        //SLOW Mode
        boolean slowMode = false;
        boolean slowmoTriggerReleased = true;

        //Increment
        boolean incrementUpReleased = true;
        boolean incrementDownReleased = true;

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
        Robot.grabberServo.setPosition(Robot.grabberServoOpenPos);
        grabberServoCurrentPos = Robot.grabberServoOpenPos;


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

            boolean uprightCone = gamepad2.left_bumper;



            //----------------------------------------------------------------//
            //GAMEPAD1 CONTROLS = Driving + slow mode + manual lift + score button

            double slowmoTrigger = gamepad1.left_trigger;

            double leftStickY = gamepad1.left_stick_y * -1 * Robot.slowModeSpeed;
            double leftStickX = gamepad1.left_stick_x * Robot.slowModeSpeed * 0.8; // testing

            if (scoreSteps > 0) {
                leftStickX = 0;
                leftStickY = 0;
            }

            double rightStickX = gamepad1.right_stick_x * Robot.slowModeTurnSpeed * .8;

            boolean liftPosUpManualButton = gamepad1.right_bumper;
            boolean liftPosDownManualButton = gamepad1.left_bumper;

            double scoreButton = gamepad1.right_trigger;

            boolean autoScoreModeButton = gamepad1.start;



            //---------------------------------------------------------------//
            //READ HARDWARE VALUES

            liftCurrentHeight = Robot.liftMotor.getCurrentPosition() / Robot.liftTicksPerInch;
            turretCurrentDegrees = Robot.turretMotor.getCurrentPosition() / Robot.turretTicksPerDegree;

            avgWheelVelocityFPS = GetAverageVelocity();

            int frontLeft = Robot.frontLeft.getCurrentPosition();
            int frontRight = Robot.frontRight.getCurrentPosition();
            int backLeft = Robot.backLeft.getCurrentPosition();
            int backRight = Robot.backRight.getCurrentPosition();





            //-----------------------------------------------------------//
            //SLO-MO CODE

            if (slowmoTrigger > Robot.triggerSensitivity) {
                if (slowmoTriggerReleased) {
                    if (!slowMode) {
                        if (Robot.liftJunctionGroundHeight == liftHeightPrevTarget) {
                            Robot.slowModeSpeed = Robot.slowModeGroundJuctionSlow;
                        } else {
                            Robot.slowModeSpeed = Robot.slowModeSlow;
                        }
                        Robot.slowModeTurnSpeed = Robot.slowModeTurnSlow;
                        slowMode = true;
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


            if (autoScoreModeButton) {
                if (autoScoreModeReleased) {
                    autoScoreMode = !autoScoreMode;
                    autoScoreModeReleased = false;
                }
            }
            else {
                autoScoreModeReleased = true;
            }

            if (!manualMode) {

                //----------------------------------------------------------//
                //LIFT MOTOR

                if (liftPosGroundButton) {
                    liftHeightTarget = Robot.liftPickupHeight;
                    lastHeightTargetNoReset = Robot.liftPickupHeight;

                    turretButtonChoiceTargetDegrees = Robot.turretForwardDegrees;

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

            //MANUAL MODE
            else {
                if (Math.abs(gamepad2.left_stick_y / 4) > 0.01) {
                    Robot.liftMotor.setPower((gamepad2.left_stick_y / 4) * -1);
                }
                else {
                    Robot.liftMotor.setPower(0);
                }
                Robot.turretMotor.setPower(gamepad2.right_stick_x / 4);
            }


            //---------------------------------------------------------------//
            //GRABBER SERVO CODE

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

            if (uprightCone) {
                Robot.grabberServo.setPosition(Robot.grabberServoUprightPos);
                liftHeightTarget = Robot.liftConeUprightHeight;
            }


            //GUIDE SERVO CODE
            if (liftCurrentHeight > Robot.guideServoDeployHeight && liftHeightTarget >= Robot.liftJunctionLowHeight && grabberServoCurrentPos == Robot.grabberServoClosedPos && scoreSteps == 0) {
                Robot.guideServo.setPosition(Robot.guideServoDown);

            }
            else {
                Robot.guideServo.setPosition(Robot.guideServoUp);
            }

            //Autoscore
            if (liftCurrentHeight > Robot.guideServoDeployHeight && autoScoreMode) {
                if (allowAutoScore) {
                    scoreSteps = CheckForPole(avgWheelVelocityFPS, lastHeightTargetNoReset, grabberServoCurrentPos, frontLeft, frontRight, backLeft, backRight, GetAveragePosition());
                    if (scoreSteps != 0) {
                        allowAutoScore = false;
                    }
                }
            }
            if (liftCurrentHeight < Robot.liftConeUprightHeight) {
                allowAutoScore = true;
            }

            //Manual Score
            if (scoreButton > Robot.triggerSensitivity) {
                scoreSteps = 1;
            }

            if (scoreSteps == 3 && dropTimer.milliseconds() > liftRaiseWaitTime) {
                sleep(250);
                liftHeightTarget = lastHeightTargetNoReset;
                scoreSteps = 0;
            }
            if (Robot.liftMotor.getTargetPosition() != 0) {
                if (scoreSteps == 2 && Robot.liftMotor.isBusy()) {
                    grabberServoCurrentPos = Robot.grabberServoOpenPos;
                    Robot.grabberServo.setPosition(grabberServoCurrentPos);
                    dropTimer.reset();
                    scoreSteps = 3;

                }
            }
            else {
                if (scoreSteps == 2 && !Robot.liftMotor.isBusy()) {
                    grabberServoCurrentPos = Robot.grabberServoOpenPos;
                    Robot.grabberServo.setPosition(grabberServoCurrentPos);
                    dropTimer.reset();
                    scoreSteps = 3;
                }
            }
            if (scoreSteps == 1) {
                Robot.guideServo.setPosition(Robot.guideServoUp);
                liftHeightTarget = lastHeightTargetNoReset - 5;
                scoreSteps = 2;
            }


            //---------------------------------------------------------//
            //Tightening Code
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

            if (liftCurrentHeight > Robot.liftJunctionLowHeight && Math.abs(avgWheelVelocityFPS) > 0.5) {
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

            //---------------------------------------------------------------------//
            //TELEMETRY CODE
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

    public int CheckForPole(double avgWheelVelocityFPS, double lastHeightTargetNoReset, double grabberServoCurrentPos, int frontLeft, int frontRight, int backLeft, int backRight, double originalAveragePosition) {

        int adjustmentAmount = (int) (avgWheelVelocityFPS * Robot.ticksPerInch * 0);
        if (Math.abs(avgWheelVelocityFPS) < 1.2) {
            if (Robot.colorSensorPole.green() > Robot.colorThreshold) {
                Robot.frontLeft.setPower(0.01);
                Robot.frontRight.setPower(0.01);
                Robot.backLeft.setPower(0.01);
                Robot.backRight.setPower(0.01);


                Robot.frontLeft.setTargetPosition(frontLeft - adjustmentAmount);
                Robot.frontRight.setTargetPosition(frontRight - adjustmentAmount);
                Robot.backLeft.setTargetPosition(backLeft - adjustmentAmount);
                Robot.backRight.setTargetPosition(backRight - adjustmentAmount);


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


                Robot.frontLeft.setPower(0.2);
                Robot.frontRight.setPower(0.2);
                Robot.backLeft.setPower(0.2);
                Robot.backRight.setPower(0.2);


                while (opModeIsActive() && Robot.frontLeft.isBusy()){ //(Math.abs(originalAveragePosition - adjustmentAmount) - Math.abs(GetAveragePosition())) * Robot.ticksPerInch > 0.25) { //loop until within quarter inch of target
                    telemetry.addData("distance travelled", (Math.abs(originalAveragePosition - adjustmentAmount) - Math.abs(GetAveragePosition())) * Robot.ticksPerInch);
                    telemetry.update();
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



                /**
                Robot.liftMotor.setTargetPosition((int) ((lastHeightTargetNoReset - 5) * Robot.liftTicksPerInch)); // last height target no reset - 5
                Robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Robot.liftMotor.setPower(0.8);

                while (opModeIsActive() && ((float) Robot.liftMotor.getCurrentPosition() / Robot.liftMotor.getTargetPosition()) < 0.98) {
                    if ((Robot.liftMotor.getCurrentPosition() / Robot.liftMotor.getTargetPosition()) > 0.8) {
                        Robot.guideServo.setPosition(Robot.guideServoUp);
                    }

                    Log.d("AutoScore", "Lift Down");
                }
                Log.d("AutoScore", "Lift Down Finished");


                grabberServoCurrentPos = Robot.grabberServoOpenPos;
                Robot.grabberServo.setPosition(grabberServoCurrentPos);

                Robot.liftMotor.setTargetPosition((int) ((lastHeightTargetNoReset) * Robot.liftTicksPerInch));
                Robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Robot.liftMotor.setPower(Robot.liftSpeedUp);
                while (opModeIsActive() && Robot.liftMotor.isBusy()) {}
                **/
                return 1;
            }
        }
        return 0;
    }

}


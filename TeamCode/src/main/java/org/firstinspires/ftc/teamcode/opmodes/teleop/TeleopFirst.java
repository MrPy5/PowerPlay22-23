package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.robot.Robot;


@TeleOp

public class TeleopFirst extends LinearOpMode {





    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap);
        waitForStart();


        double slowfactor = 1;
        boolean slow_mode = false;
        boolean slowmoTriggerReleased = true;


        //---------------------------------------------------------------//
        //GRABBER SERVO VARIABLES

        double grabberServoClosedPos = 0.7;
        double grabberServoOpenPos = 0.3;
        double grabberServoCurrentPos = 0;
        boolean grabberTriggerReleased = true;



        //---------------------------------------------------------------//
        //GRABBER TURRET VARIABLES

        double turretSpeed = 0.5;

        //         0
        //      |     |
        //    3    ^     1
        //      |     |
        //         2

        int forwardTurretPosTicks = 0;
        int rightTurretPosTicks = 696;
        int leftTurretPosTicks = -696;
        int backTurretPosTicks = 1393;


        //---------------------------------------------------------------//
        //LIFT VARIABLES

        int liftPos = 0;
        double liftSpeed = 0.5;

        //   >____
        //    3  |  High
        //    2  |  Medium
        //    1  |  Low
        //    0  |  Ground
        //


        int ticksPerRevolutionOrbital = 537;

        int groundLiftPosTicks = 0;
        int overChassisLiftPosTicks = ticksPerRevolutionOrbital * 1;
        int lowLiftPosTicks = ticksPerRevolutionOrbital * 2;
        int mediumLiftPosTicks = ticksPerRevolutionOrbital * 3;
        int highLiftPosTicks = ticksPerRevolutionOrbital * 5;







        while (opModeIsActive()) {

            //---------------------------------------------------------------//
            //GAMEPAD2 CONTROLS

            double grabberTrigger = gamepad2.right_trigger;

            boolean forwardButton = gamepad2.dpad_up;
            boolean backButton = gamepad2.dpad_down;
            boolean leftButton = gamepad2.dpad_left;
            boolean rightButton = gamepad2.dpad_right;

            boolean groundButton = gamepad2.x;
            boolean lowButton = gamepad2.a;
            boolean mediumButton = gamepad2.b;
            boolean highButton = gamepad2.y;

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

            if (slowmoTrigger > 0.1 && slowmoTriggerReleased) {

                slowmoTriggerReleased = false;
                if (slow_mode == false) {
                    slowfactor = 0.3;
                    slow_mode = true;
                }

                else if (slow_mode) {
                    slowfactor = 1;
                    slow_mode = false;
                }
            }

            else {
                slowmoTriggerReleased = true;
            }

            //---------------------------------------------------------------//
            //GRABBER SERVO CODE

            if (grabberTrigger > 0.01 && grabberTriggerReleased) {
                if (grabberServoCurrentPos == grabberServoOpenPos) {
                    grabberServoCurrentPos = grabberServoClosedPos;
                }
                else {
                    grabberServoCurrentPos = grabberServoOpenPos;
                }
                Robot.gripperServo.setPosition(grabberServoCurrentPos);
                grabberTriggerReleased = false;
            } else {
                grabberTriggerReleased = true;
            }


            //---------------------------------------------------------------//
            //TURRET CODE

            if (forwardButton) {
                robot.turretMotor.setTargetPosition(forwardTurretPosTicks);
                robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.turretMotor.setPower(turretSpeed * slowfactor);



            }
            if (leftButton) {
                robot.turretMotor.setTargetPosition(leftTurretPosTicks);
                robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.turretMotor.setPower(turretSpeed * slowfactor);



            }
            if (rightButton) {
                robot.turretMotor.setTargetPosition(rightTurretPosTicks);
                robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.turretMotor.setPower(turretSpeed * slowfactor);



            }
            if (backButton) {
                robot.turretMotor.setTargetPosition(backTurretPosTicks);
                robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.turretMotor.setPower(turretSpeed * slowfactor);



            }


            //----------------------------------------------------------//
            //TURRET + LIFT INTEGRATION

            if (liftPos == 0 && (backButton || forwardButton || leftButton || rightButton)) {
                robot.liftMotor.setTargetPosition(overChassisLiftPosTicks);
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftMotor.setPower(0.75);
            }



            if (!robot.turretMotor.isBusy()) {

               //INTEGRATION with Battery
                if (robot.turretMotor.getPower() > 0 && liftPos == 0) {
                    robot.liftMotor.setTargetPosition(0);
                    robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotor.setPower(0.75);
                }
                robot.turretMotor.setPower(0);
            }



            //----------------------------------------------------------//
            //LIFT MOTOR

            if (groundButton) {
                robot.liftMotor.setTargetPosition(groundLiftPosTicks);
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftMotor.setPower(liftSpeed * slowfactor);

                liftPos = 0;

            }

            if (lowButton) {
                robot.liftMotor.setTargetPosition(lowLiftPosTicks);
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftMotor.setPower(liftSpeed * slowfactor);

                liftPos = 1;

            }

            if (mediumButton) {
                robot.liftMotor.setTargetPosition(mediumLiftPosTicks);
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftMotor.setPower(liftSpeed * slowfactor);

                liftPos = 2;
            }
            if (highButton) {
                robot.liftMotor.setTargetPosition(highLiftPosTicks);
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftMotor.setPower(liftSpeed * slowfactor);

                liftPos = 3;

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

            robot.backLeft.setPower(lrPower);
            robot.backRight.setPower(rrPower);
            robot.frontLeft.setPower(lfPower);
            robot.frontRight.setPower(rfPower);




            //---------------------------------------------------------------------//
            //TELEMETRY CODE



            telemetry.update();
        }
    }
}

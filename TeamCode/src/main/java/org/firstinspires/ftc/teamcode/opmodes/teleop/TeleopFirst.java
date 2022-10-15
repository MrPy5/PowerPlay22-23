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
        boolean reset_slow = true;


        //---------------------------------------------------------------//
        //GRABBER SERVO VARIABLES

        double grabberServoClosedPos = 0.7;
        double grabberServoOpenPos = 0.3;
        double grabberServoCurrentPos = 0;
        boolean grabberTriggerReleased = true;

        //       0 = open
        //       1 = closed


        //---------------------------------------------------------------//
        //GRABBER TURRET VARIABLES

        int turretPos = 0;

        //         0
        //      |     |
        //    3    ^     1
        //      |     |
        //         2

        int forwardTurretPosTicks = 0;
        int rightTurretPosTicks = 696;
        int leftTurretPosTicks = -696;
        int backTurretPosTicks = 1393;


        // *** Lift ***

        int liftPos = 0;

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
            //GAMEPAD1 CONTROLs
            double grabberTrigger = gamepad2.right_trigger;



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

            if (gamepad1.right_trigger > 0.1 && reset_slow) {

                reset_slow = false;
                if (slow_mode == false) {
                    slowfactor = 0.3;
                    slow_mode = true;
                }
                else if (slow_mode) {
                    slowfactor = 1;
                    slow_mode = false;
                }
            }

            if (gamepad1.right_trigger <= 0.1) {
                reset_slow = true;
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

            if (gamepad2.dpad_up) {
                robot.turretMotor.setTargetPosition(forwardTurretPosTicks);
                robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.turretMotor.setPower(0.5 * slowfactor);

                turretPos = 0;

            }
            if (gamepad2.dpad_left) {
                robot.turretMotor.setTargetPosition(leftTurretPosTicks);
                robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.turretMotor.setPower(0.5 * slowfactor);

                turretPos = 3;

            }
            if (gamepad2.dpad_right) {
                robot.turretMotor.setTargetPosition(rightTurretPosTicks);
                robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.turretMotor.setPower(0.5 * slowfactor);

                turretPos = 1;

            }
            if (gamepad2.dpad_down) {
                robot.turretMotor.setTargetPosition(backTurretPosTicks);
                robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.turretMotor.setPower(0.5 * slowfactor);

                turretPos = 2;

            }


            //----------------------------------------------------------//
            //TURRET + LIFT INTEGRATION

            if (liftPos == 0 && (gamepad2.dpad_down || gamepad2.dpad_up || gamepad2.dpad_left || gamepad2.dpad_right)) {
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

            if (gamepad2.x) {
                robot.liftMotor.setTargetPosition(groundLiftPosTicks);
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftMotor.setPower(0.3 * slowfactor);

                liftPos = 0;

            }

            if (gamepad2.a) {
                robot.liftMotor.setTargetPosition(lowLiftPosTicks);
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftMotor.setPower(0.3 * slowfactor);

                liftPos = 1;

            }

            if (gamepad2.b) {
                robot.liftMotor.setTargetPosition(mediumLiftPosTicks);
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftMotor.setPower(0.3 * slowfactor);

                liftPos = 2;
            }
            if (gamepad2.y) {
                robot.liftMotor.setTargetPosition(highLiftPosTicks);
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftMotor.setPower(0.3 * slowfactor);

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

            telemetry.addData("LeftStickX", gamepad1.left_stick_x);
            telemetry.addData("LeftStickY", gamepad1.left_stick_y);
            telemetry.addData("RightStickX", gamepad1.right_stick_x);
            telemetry.addData("RightStickY", grabberTriggerReleased);

            telemetry.update();
        }
    }
}

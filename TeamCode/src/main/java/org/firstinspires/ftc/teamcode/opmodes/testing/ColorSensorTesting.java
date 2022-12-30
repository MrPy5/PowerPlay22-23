//Checklist
/*

-180 turn for left
-


 */


package org.firstinspires.ftc.teamcode.opmodes.testing;

import android.util.Log;

//import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.robot.Robot;

import java.util.List;


@TeleOp(name = "Color Sensor Testing With Pole")

public class ColorSensorTesting extends LinearOpMode {


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
        robot.grabberServo.setPosition(robot.grabberServoClosedPos);
        grabberServoCurrentPos = robot.grabberServoClosedPos;

        sleep(2000);

        robot.liftMotor.setTargetPosition((int) (robot.liftJunctionHighHeight * robot.liftTicksPerInch));
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setPower(robot.liftSpeedUp);
        robot.guideServo.setPosition(robot.guideServoDown);

        sleep(1000);


        robot.turretMotor.setTargetPosition((int) (robot.turretRightDegrees * robot.turretTicksPerDegree));
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.turretMotor.setPower(robot.turretSpeed);

        sleep(1000);

        while (opModeIsActive()) {

            //---------------------------------------------------------------//
            //READ HARDWARE VALUES

            double leftStickY = gamepad1.left_stick_y * -1 * robot.slowModeSpeed;
            double leftStickX = gamepad1.left_stick_x * robot.slowModeSpeed * 0.8; // testing
            double rightStickX = gamepad1.right_stick_x * robot.slowModeTurnSpeed * .8;

            avgWheelVelocityFPS = GetAverageVelocity(robot);



            //Check for Pole



            //Driving
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

            robot.backLeft.setPower(lrPower);
            robot.backRight.setPower(rrPower);
            robot.frontLeft.setPower(lfPower);
            robot.frontRight.setPower(rfPower);

            telemetry.addData("LeftStickX:", gamepad1.left_stick_x);
            telemetry.addData("LeftStickY:", gamepad1.left_stick_y);

            telemetry.addData("WheelPower:", wheelPower);

            telemetry.addData("Average Velocity:", GetAverageVelocity(robot));
            telemetry.addData("Distance", robot.colorSensorPole.green());
            telemetry.addData("Steps", scoreSteps);
            telemetry.addData("green", robot.colorSensorPole.green());
            telemetry.update();


        }
    }

    public double GetAverageVelocity(Robot robot) {
        double averageVelocity;
        averageVelocity = (robot.backRight.getVelocity() + robot.backLeft.getVelocity() + robot.frontLeft.getVelocity() + robot.frontRight.getVelocity()) / 4;
        averageVelocity = (averageVelocity / robot.ticksPerInch) / 12;
        return averageVelocity;
    }




}


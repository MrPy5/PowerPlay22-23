//Checklist
/*

-180 turn for left
-


 */


package org.firstinspires.ftc.teamcode.opmodes.testing;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
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

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

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
            robot.liftMotor.setTargetPosition((int) (robot.liftJunctionHighHeight * robot.liftTicksPerInch));
            robot.liftMotor.setPower(robot.liftSpeedUp);


            //---------------------------------------------------------------//
            //READ HARDWARE VALUES

            liftCurrentHeight = robot.liftMotor.getCurrentPosition() / robot.liftTicksPerInch;
            turretCurrentDegrees = robot.turretMotor.getCurrentPosition() / robot.turretTicksPerDegree;

            avgWheelVelocityFPS = GetAverageVelocity(robot);



            grabberServoCurrentPos = CheckForPole(robot, 1, robot.liftJunctionHighHeight, grabberServoCurrentPos, robot.frontLeft.getCurrentPosition(), robot.frontRight.getCurrentPosition(), robot.backLeft.getCurrentPosition(), robot.backRight.getCurrentPosition());

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
                robot.frontLeft.setTargetPosition(frontLeft);
                robot.frontRight.setTargetPosition(frontRight);
                robot.backLeft.setTargetPosition(backLeft);
                robot.backRight.setTargetPosition(backRight);

                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.frontLeft.setPower(0.8);
                robot.frontRight.setPower(0.8);
                robot.backLeft.setPower(0.8);
                robot.backRight.setPower(0.8);

                robot.guideServo.setPosition(robot.guideServoUp);

                robot.liftMotor.setTargetPosition((int) ((lastHeightTargetNoReset) * robot.liftTicksPerInch));
                robot.liftMotor.setPower(0.8);
                while (opModeIsActive() && (robot.frontLeft.isBusy() || robot.frontRight.isBusy() || robot.backLeft.isBusy() || robot.backRight.isBusy() || robot.liftMotor.isBusy())) {}

                robot.liftMotor.setTargetPosition((int) ((lastHeightTargetNoReset - 5) * robot.liftTicksPerInch));
                robot.liftMotor.setPower(0.8);

                while (opModeIsActive() && robot.liftMotor.isBusy()) {}

                grabberServoCurrentPos = robot.grabberServoOpenPos;
                robot.grabberServo.setPosition(grabberServoCurrentPos);

                robot.liftMotor.setTargetPosition((int) ((lastHeightTargetNoReset) * robot.liftTicksPerInch));
                robot.liftMotor.setPower(robot.liftSpeedUp);
                while (opModeIsActive() && robot.liftMotor.isBusy()) {}

                robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
        return grabberServoCurrentPos;
    }

}


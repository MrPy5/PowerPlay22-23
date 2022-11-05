/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.robot.Robot;
import org.firstinspires.ftc.teamcode.opmodes.teleop.TeleopFirst;

import java.util.Locale;


@Autonomous(name="Auto")

public class Auto extends LinearOpMode {

    /* Declare OpMode members. */
    BNO055IMU imu;
    Orientation angles;

    //drive
    double countsPerInch = Robot.ticksPerInch;
    double driveSpeedCurrent = 0.3;
    double driveSpeedFast = 0.4;

    //turret
    double turretSpeed = 0.5;
    double turretCurrentDegrees;
    double turretCloseToZero = 70;

    double turretTicksPerDegree = Robot.turretTicksPerDegree;

    public static double turretForwardDegrees = 0; //all rotation variables in degrees
    public static double turretRightDegrees = 90;
    public static double turretLeftDegrees = -90;
    public static double turretBackDegrees = 180;

    public static double turretTargetDegrees = 0;
    double turretPrevTargetDegrees = 0;

    //lift
    double liftSpeedUp = 1;
    double liftSpeedDown = .5;
    double liftSpeedPower;

    double liftTicksPerInch = Robot.liftTicksPerInch;

    double liftCurrentHeight; //all height variables are in inches
    double liftPickupHeight = 0;
    double liftJunctionGroundHeight = 2;
    double liftJunctionLowHeight = 15;
    double liftJunctionMediumHeight = 24;
    double liftJunctionHighHeight = 34;
    double liftMinHeightForTurning = 6;
    double liftMaximumHeight = 36;

    double liftHeightTarget = 0;
    double liftHeightPrevTarget = 0;

    //grabber
    double grabberServoClosedPos = Robot.grabberServoClosedPos;
    double grabberServoOpenPos = Robot.grabberServoOpenPos;
    double grabberServoCurrentPos = 0.22;
    float changeFromZero = 0;
    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        waitForStart();
        sleep(5000);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float currentHeading = Float.parseFloat(formatAngle(angles.angleUnit, angles.firstAngle));
        changeFromZero = (float) currentHeading;

        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);

        ZeroPowerToBrake();
        Drive(65);


        RaiseLift(Robot.liftJunctionHighHeight, true);


        TurnTurret(Robot.turretRightDegrees, true);


        sleep(1000);

        RaiseLift(Robot.liftJunctionHighHeight - 2, true);

        Robot.grabberServo.setPosition(Robot.grabberServoOpenPos);

        RaiseLift(Robot.liftJunctionHighHeight, true);

        TurnTurret(Robot.turretForwardDegrees, true);

        RaiseLift(0, true);

        Drive(-12);

        Turn(90);

        RaiseLift(7, false);

        Drive(24);




    }

    
    public void Drive(int inches) {
        ResetEncoders();
        Robot.frontLeft.setTargetPosition((int) (inches * countsPerInch));
        Robot.frontRight.setTargetPosition((int) (inches * countsPerInch));
        Robot.backLeft.setTargetPosition((int) (inches * countsPerInch));
        Robot.backRight.setTargetPosition((int) (inches * countsPerInch));

        Robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Robot.frontLeft.setPower(driveSpeedCurrent);
        Robot.frontRight.setPower(driveSpeedCurrent);
        Robot.backLeft.setPower(driveSpeedCurrent);
        Robot.backRight.setPower(driveSpeedCurrent);

        while (Robot.frontLeft.isBusy() || Robot.frontRight.isBusy() || Robot.backLeft.isBusy() || Robot.backRight.isBusy()) {
            if (Robot.frontLeft.getCurrentPosition() / (inches * countsPerInch) <= 0.3) {
                if (driveSpeedCurrent < driveSpeedFast) {
                    driveSpeedCurrent = driveSpeedCurrent + 0.01;
                }
            }
            if (Robot.frontLeft.getCurrentPosition() / (inches * countsPerInch) >= 0.7) {
                if (driveSpeedCurrent > 0.1) {
                    driveSpeedCurrent = driveSpeedCurrent - 0.01;
                }
                else {
                    driveSpeedCurrent = 0.1;
                }
            }

            Robot.frontLeft.setPower(driveSpeedCurrent);
            Robot.frontRight.setPower(driveSpeedCurrent);
            Robot.backLeft.setPower(driveSpeedCurrent);
            Robot.backRight.setPower(driveSpeedCurrent);




        }

        Robot.frontLeft.setPower(0);
        Robot.frontRight.setPower(0);
        Robot.backLeft.setPower(0);
        Robot.backRight.setPower(0);
    }

    public void Turn(double targetAngle) {
        ResetEncoders();
        Robot.frontLeft.setPower(-0.05);
        Robot.frontRight.setPower(0.05);
        Robot.backLeft.setPower(-0.05);
        Robot.backRight.setPower(0.05);
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double distance = Math.abs(Float.parseFloat(formatAngle(angles.angleUnit, angles.firstAngle)) - changeFromZero - targetAngle);
        while ((int) (Float.parseFloat(formatAngle(angles.angleUnit, angles.firstAngle)) + changeFromZero) > targetAngle + 0.5 || (int) (Float.parseFloat(formatAngle(angles.angleUnit, angles.firstAngle)) + changeFromZero) < targetAngle - 0.5) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("> ", Float.parseFloat(formatAngle(angles.angleUnit, angles.firstAngle))-changeFromZero);
            telemetry.update();




        }



        Robot.frontLeft.setPower(0);
        Robot.frontRight.setPower(0);
        Robot.backLeft.setPower(0);
        Robot.backRight.setPower(0);
    }
    public void TurnTurret(double turretTargetDegrees, boolean wait) {
        liftCurrentHeight = Robot.liftMotor.getCurrentPosition() / liftTicksPerInch;
        turretCurrentDegrees = Robot.turretMotor.getCurrentPosition() / turretTicksPerDegree;



        if (liftCurrentHeight > liftMinHeightForTurning) {
            if (turretTargetDegrees != turretPrevTargetDegrees) {
                Robot.turretMotor.setTargetPosition((int) (turretTargetDegrees * turretTicksPerDegree));
                Robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Robot.turretMotor.setPower(turretSpeed);
                turretPrevTargetDegrees = turretTargetDegrees;
            }
            else if (Math.abs(turretTargetDegrees - turretCurrentDegrees) > turretCloseToZero) {  // If not close to destination, temporarily keep turret where it is until lift raises.
                Robot.liftMotor.setTargetPosition((int) (liftMinHeightForTurning * liftTicksPerInch));
                Robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Robot.liftMotor.setPower(liftSpeedUp);
                turretPrevTargetDegrees = turretCurrentDegrees;
            }
        }

        if (wait) {
            while(Robot.turretMotor.isBusy());
            Robot.turretMotor.setPower(0);
        }
    }

    public void RaiseLift(double liftHeightTarget, boolean wait) {
        liftCurrentHeight = Robot.liftMotor.getCurrentPosition() / liftTicksPerInch;
        turretCurrentDegrees = Robot.turretMotor.getCurrentPosition() / turretTicksPerDegree;

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
        if (wait) {
            while (Robot.liftMotor.isBusy());
            Robot.liftMotor.setPower(0);
        }
    }
    public void ChangeGripperState(double gripperTarget) {
        if (grabberServoCurrentPos == grabberServoOpenPos) {
            grabberServoCurrentPos = grabberServoClosedPos;
        } else {
            grabberServoCurrentPos = grabberServoOpenPos;
        }
        Robot.grabberServo.setPosition(grabberServoCurrentPos);
    }

    public void ResetEncoders() {
        Robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void ZeroPowerToBrake() {
        Robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    public void ZeroPowerToFloat() {
        Robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}

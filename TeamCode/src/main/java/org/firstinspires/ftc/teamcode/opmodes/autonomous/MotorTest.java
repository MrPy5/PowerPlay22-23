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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.robot.Robot;


import java.util.Locale;


@Autonomous(name="MotorTest")

public class MotorTest extends LinearOpMode {

    /* Declare OpMode members. */



    static final double     COUNTS_PER_MOTOR_REV    = 537.6;
    static final double     WHEEL_DIAMETER_INCHES   = 3.77;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    double     driveSpeed             = 0.3;

    BNO055IMU imu;
    Orientation angles;
    public static float changeFromZero = 0;
    @Override
    public void runOpMode() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        Robot robot = new Robot(hardwareMap);

        waitForStart();

        ResetEncoders();
        /*while (opModeIsActive()) {
            Robot.frontLeft.setPower(driveSpeed);
            Robot.frontRight.setPower(driveSpeed);
            Robot.backLeft.setPower(driveSpeed);
            Robot.backRight.setPower(driveSpeed);
            telemetry.addData("FrontLeft", Robot.frontLeft.getCurrentPosition());
            telemetry.addData("FrontRight", Robot.frontRight.getCurrentPosition());
            telemetry.addData("BackLeft", Robot.backLeft.getCurrentPosition());
            telemetry.addData("BackRight", Robot.backRight.getCurrentPosition());
            telemetry.update();
        }*/
        /*Turn(180);
        sleep(1000);
        Turn(90);
        sleep(1000);
        Turn(-90);
        sleep(1000);
        Turn(-360);
        sleep(1000);
        */
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float currentHeading = angles.firstAngle;
        changeFromZero = (float) currentHeading;

        Turn(270);
        sleep(1000);
        Turn(90);
        sleep(1000);
        Turn(180);
    }




    public void Turn(double targetAngle) {
        ResetEncoders();

        boolean goRight = false;
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double distance = Math.abs(angles.firstAngle - changeFromZero - targetAngle);
        double currentAngle =  angles.firstAngle - changeFromZero;
        if (currentAngle < 0) {
            currentAngle = 360 + currentAngle;
        }
        telemetry.addData(">", currentAngle - targetAngle);
        telemetry.update();
        double degreesToTurn = Math.abs(targetAngle - currentAngle);

        goRight = targetAngle > currentAngle;

        if (degreesToTurn > 180) {
            goRight = !goRight;
            degreesToTurn = 360 - degreesToTurn;
        }

        if (goRight) {
            Robot.frontLeft.setPower(-0.2);
            Robot.frontRight.setPower(0.2);
            Robot.backLeft.setPower(-0.2);
            Robot.backRight.setPower(0.2);
        }

        else {
            Robot.frontLeft.setPower(0.2);
            Robot.frontRight.setPower(-0.2);
            Robot.backLeft.setPower(0.2);
            Robot.backRight.setPower(-0.2);
        }


        while ((currentAngle > targetAngle + 30 || currentAngle < targetAngle - 30) && opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentAngle =  angles.firstAngle - changeFromZero;
            if (currentAngle < 0) {
                currentAngle = 360 + currentAngle;
            }





        }
        
        if (goRight) {
            Robot.frontLeft.setPower(-0.05);
            Robot.frontRight.setPower(0.05);
            Robot.backLeft.setPower(-0.05);
            Robot.backRight.setPower(0.05);
        }

        else {
            Robot.frontLeft.setPower(0.05);
            Robot.frontRight.setPower(-0.05);
            Robot.backLeft.setPower(0.05);
            Robot.backRight.setPower(-0.05);
        }
        
        while ((currentAngle > targetAngle + 0.5 || currentAngle < targetAngle - 0.5) && opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentAngle =  angles.firstAngle- changeFromZero;
            if (currentAngle < 0) {
                currentAngle = 360 + currentAngle;
            }
            telemetry.addData("> ", currentAngle);
            telemetry.addData("> ", angles.firstAngle - changeFromZero);
            telemetry.update();

        }

        Robot.frontLeft.setPower(0);
        Robot.frontRight.setPower(0);
        Robot.backLeft.setPower(0);
        Robot.backRight.setPower(0);
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
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}

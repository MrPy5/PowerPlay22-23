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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.hardware.robot.Robot;



@Autonomous(name="Auto")

public class Auto extends LinearOpMode {

    /* Declare OpMode members. */



    static final double     COUNTS_PER_MOTOR_REV    = 537.6;
    static final double     WHEEL_DIAMETER_INCHES   = 3.77;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    double     DRIVE_SPEED             = 0.3;

    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap);

        waitForStart();

        ResetEncoders();
        ZeroPowerToFloat();
        Drive(110);

        sleep(5000);

    }

    public void Drive(int inches) {
        Robot.frontLeft.setTargetPosition((int) (inches * COUNTS_PER_INCH));
        Robot.frontRight.setTargetPosition((int) (inches * COUNTS_PER_INCH));
        Robot.backLeft.setTargetPosition((int) (inches * COUNTS_PER_INCH));
        Robot.backRight.setTargetPosition((int) (inches * COUNTS_PER_INCH));

        Robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Robot.frontLeft.setPower(DRIVE_SPEED);
        Robot.frontRight.setPower(DRIVE_SPEED);
        Robot.backLeft.setPower(DRIVE_SPEED);
        Robot.backRight.setPower(DRIVE_SPEED);

        while (Robot.frontLeft.isBusy() || Robot.frontRight.isBusy() || Robot.backLeft.isBusy() || Robot.backRight.isBusy()) {
            if (Robot.frontLeft.getCurrentPosition() / (inches * COUNTS_PER_INCH) <= 0.3) {
                if (DRIVE_SPEED < 0.8) {
                    DRIVE_SPEED = DRIVE_SPEED + 0.01;
                }
            }
            if (Robot.frontLeft.getCurrentPosition() / (inches * COUNTS_PER_INCH) >= 0.7) {
                if (DRIVE_SPEED > 0.1) {
                    DRIVE_SPEED = DRIVE_SPEED - 0.01;
                }
                else {
                    DRIVE_SPEED = 0.1;
                }
            }

            Robot.frontLeft.setPower(DRIVE_SPEED);
            Robot.frontRight.setPower(DRIVE_SPEED);
            Robot.backLeft.setPower(DRIVE_SPEED);
            Robot.backRight.setPower(DRIVE_SPEED);

            telemetry.addData("speed", DRIVE_SPEED);
            telemetry.update();


        }

        Robot.frontLeft.setPower(0);
        Robot.frontRight.setPower(0);
        Robot.backLeft.setPower(0);
        Robot.backRight.setPower(0);
    }

    public void Turn(int inches_left, int inches_right) {
        Robot.frontLeft.setTargetPosition((int) (inches_left * COUNTS_PER_INCH));
        Robot.frontRight.setTargetPosition((int) (inches_right * COUNTS_PER_INCH));
        Robot.backLeft.setTargetPosition((int) (inches_left * COUNTS_PER_INCH));
        Robot.backRight.setTargetPosition((int) (inches_right * COUNTS_PER_INCH));

        Robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Robot.frontLeft.setPower(DRIVE_SPEED);
        Robot.frontRight.setPower(DRIVE_SPEED);
        Robot.backLeft.setPower(DRIVE_SPEED);
        Robot.backRight.setPower(DRIVE_SPEED);

        while (Robot.frontLeft.isBusy() || Robot.frontRight.isBusy() || Robot.backLeft.isBusy() || Robot.backRight.isBusy()) {

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

}

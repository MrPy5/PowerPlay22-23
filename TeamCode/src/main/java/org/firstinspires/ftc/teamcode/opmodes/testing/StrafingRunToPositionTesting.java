//Checklist
/*

-180 turn for left
-


 */


package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.robot.Robot;


@TeleOp(name = "Strafe Testing")

public class StrafingRunToPositionTesting extends LinearOpMode {


    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap);
        waitForStart();


        double triggerSensitivity = 0.01;
        
        double targetPosition = 0;
        double driveSpeedCurrent = 0.3;

        boolean gamepadAReleased = true;
        boolean gamepadBReleased = true;

        waitForStart();
        
        ResetEncoders();
        ZeroPowerToBrake();
        
        while (opModeIsActive()) {
            telemetry.addData("Press A to add 100 to target ticks, B To subtract 100. Press the right bumper to move right and the left bumper to move left", "");
            if (gamepadAReleased && gamepad1.a) {

                targetPosition += 100;
                gamepadAReleased = false;
            }
            if (gamepad1.a == false) {
                gamepadAReleased = true;
            }
            if (gamepadBReleased && gamepad1.b) {

                targetPosition -= 100;
                gamepadBReleased = false;
            }
            if (gamepad1.b == false) {
                gamepadBReleased = true;
            }

            if (gamepad1.right_bumper) {
                    ResetEncoders();
                    telemetry.addData("Moving right to", targetPosition);


                    Robot.frontLeft.setTargetPosition((int) targetPosition);
                    Robot.frontRight.setTargetPosition((int) targetPosition);
                    Robot.backLeft.setTargetPosition((int) targetPosition);
                    Robot.backRight.setTargetPosition((int) targetPosition);

                    Robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    Robot.frontLeft.setPower(driveSpeedCurrent);
                    Robot.frontRight.setPower(-driveSpeedCurrent);
                    Robot.backLeft.setPower(-driveSpeedCurrent);
                    Robot.backRight.setPower(driveSpeedCurrent);


                
            }

            if (gamepad1.left_bumper) {
                ResetEncoders();
                telemetry.addData("Moving left to", targetPosition);


                Robot.frontLeft.setTargetPosition((int) targetPosition);
                Robot.frontRight.setTargetPosition((int) targetPosition);
                Robot.backLeft.setTargetPosition((int) targetPosition);
                Robot.backRight.setTargetPosition((int) targetPosition);

                Robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                Robot.frontLeft.setPower(-driveSpeedCurrent);
                Robot.frontRight.setPower(driveSpeedCurrent);
                Robot.backLeft.setPower(driveSpeedCurrent);
                Robot.backRight.setPower(-driveSpeedCurrent);



            }
            telemetry.addData("Target Pos", targetPosition);
            telemetry.update();
            while (Robot.frontLeft.isBusy() || Robot.frontRight.isBusy() || Robot.backLeft.isBusy() || Robot.backRight.isBusy()) {
                telemetry.addData("Moving...", "");
                telemetry.update();

            }

            Robot.frontLeft.setPower(0);
            Robot.frontRight.setPower(0);
            Robot.backLeft.setPower(0);
            Robot.backRight.setPower(0);

            
        }
    }

    public static void ZeroPowerToBrake() {
        Robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static void ResetEncoders() {
        Robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    
    

}


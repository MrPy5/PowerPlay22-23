package org.firstinspires.ftc.teamcode.opmodes.game.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.hardware.robot.Robot;

@TeleOp(name = "PID test")

public class PIDtester extends LinearOpMode {


    @Override
    public void runOpMode() {
        new Robot(hardwareMap, true);
        waitForStart();


        double targetPosition = 0;
        double liftSpeedPower = 0;
        double liftCurrentHeight = 0;

        //Lift PID
        PIDFCoefficients liftPID = new PIDFCoefficients();
        liftPID = Robot.liftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        //Motor PIDs
        PIDFCoefficients frontLeftPID = new PIDFCoefficients();
        PIDFCoefficients frontRightPID = new PIDFCoefficients();
        PIDFCoefficients backLeftPID = new PIDFCoefficients();
        PIDFCoefficients backRightPID = new PIDFCoefficients();

        while (opModeIsActive()) {

            liftCurrentHeight = Robot.liftMotor.getCurrentPosition() / Robot.liftTicksPerInch;

            if (gamepad1.y) {
                targetPosition = Robot.liftJunctionHighHeight;
            }
            if (gamepad1.b) {
                targetPosition = Robot.liftJunctionMediumHeight;
            }
            if (gamepad1.a) {
                targetPosition = Robot.liftJunctionLowHeight;
            }

            Robot.liftMotor.setTargetPosition((int) (targetPosition * Robot.liftTicksPerInch));
            Robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (liftCurrentHeight < targetPosition) {
                liftSpeedPower = Robot.liftSpeedUp;
            } else {
                liftSpeedPower = Robot.liftSpeedDown;
            }
            Robot.liftMotor.setPower(liftSpeedPower);


            //Set PID
            if (gamepad1.touchpad_finger_2 && gamepad1.touchpad_finger_1) {
                Robot.liftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, liftPID);
            }

            if (gamepad1.dpad_up) {
                liftPID.p += 0.01;
            }

            if (gamepad1.dpad_down) {
                liftPID.p -= 0.01;
            }
            telemetry.addData("PID: ", "P: " + (Robot.liftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p) + " I: " + (Robot.liftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).i) + " D: " + (Robot.liftMotor.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).d));
            telemetry.update();

        }
    }
}
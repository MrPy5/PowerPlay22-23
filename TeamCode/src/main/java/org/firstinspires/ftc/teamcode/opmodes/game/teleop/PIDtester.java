package org.firstinspires.ftc.teamcode.opmodes.game.teleop;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.robot.Robot;

import java.util.List;

@TeleOp(name = "PID test")

public class PIDtester extends LinearOpMode {


    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, true);
        waitForStart();


        double targetPosition = 0;
        double liftSpeedPower = 0;
        double liftCurrentHeight = 0;


        PIDCoefficients pid = new PIDCoefficients();
        pid = Robot.liftMotor.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

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
                Robot.liftMotor.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
            }

            if (gamepad1.dpad_up) {
                pid.p += 0.01;
            }

            if (gamepad1.dpad_down) {
                pid.p -= 0.01;
            }
            telemetry.addData("PID: ", "P: " + (Robot.liftMotor.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p) + " I: " + (Robot.liftMotor.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).i) + " D: " + (Robot.liftMotor.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).d));
            telemetry.update();

        }
    }
}
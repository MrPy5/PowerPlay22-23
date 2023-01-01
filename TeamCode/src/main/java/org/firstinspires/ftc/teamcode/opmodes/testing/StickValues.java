package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.robot.Robot;


@TeleOp(name = "Stick Values")

public class StickValues extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("LeftStickX:", gamepad1.left_stick_x);
            telemetry.addData("LeftStickY:", gamepad1.left_stick_y);


            telemetry.update();

        }
    }


}




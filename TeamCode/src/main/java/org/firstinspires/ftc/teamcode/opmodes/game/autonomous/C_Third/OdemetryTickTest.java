package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.C_Third;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.robot.Robot;

@Autonomous(name="TickTest")
@Disabled
public class OdemetryTickTest extends LinearOpMode {
    BNO055IMU imu;
    Orientation angles;


    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, false);

        waitForStart();

        while (getAverageOdometerPosition() < 20) {
            Robot.frontLeft.setPower(0.1);
            Robot.frontRight.setPower(0.1);
            Robot.backLeft.setPower(0.1);
            Robot.backRight.setPower(0.1);
            telemetry.addData("Average", getAverageOdometerPosition());
            telemetry.addData("Left", Robot.odometerLeft.getCurrentPosition());
            telemetry.addData("Right", Robot.odometerRight.getCurrentPosition());
            telemetry.addData("Inches", getAverageOdometerPosition() / Robot.odometerTicksPerInch);
            telemetry.update();
        }


        }


    public double getAverageOdometerPosition() {
        return ((Robot.odometerLeft.getCurrentPosition() + Robot.odometerRight.getCurrentPosition()) / 2.0) / Robot.odometerTicksPerInch;
    }
    public double GetAverageVelocity() {
        double averageVelocity = 0;
        averageVelocity = (Robot.odometerLeft.getVelocity() + Robot.odometerRight.getVelocity()) / 2;
        averageVelocity = (averageVelocity / Robot.ticksPerInch) / 12;
        return averageVelocity;
    }


}

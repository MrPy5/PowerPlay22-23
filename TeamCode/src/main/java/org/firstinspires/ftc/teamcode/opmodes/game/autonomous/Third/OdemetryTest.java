package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.Third;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.robot.Robot;

@Autonomous(name="OdometryTest")
@Disabled
public class OdemetryTest extends LinearOpMode {
    BNO055IMU imu;
    Orientation angles;
    public double multiplier = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        initIMU();
        multiplier = getVoltageMultiplier();
        Robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        driveToPoint(40, 0, 7, 9);
        sleep(1000);


        driveToPoint(12, 0, 3, 9);
        sleep(1000);

        driveToPoint(24, 87, 5, 9);
        sleep(1000);

        driveToPoint(-37, 87, 7, 9);
        sleep(1000);


        driveToPoint(37, 87, 7, 9);
        sleep(1000);

        driveToPoint(-37, 87, 7, 9);
        sleep(1000);

        driveToPoint(37, 87, 7, 9);
        sleep(1000);

        driveToPoint(-37, 87, 7, 9);
        sleep(1000);

        driveToPoint(37, 87, 7, 9);
        sleep(1000);

        driveToPoint(-37, 87, 7, 9);
        sleep(1000);

        driveToPoint(-35, 87, 7, 9);
        sleep(1000);




    }

    public void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis
        imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);  // puts it in config mode
        sleep(100);
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE & 0x0F);
        imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);

    }
    public double getVoltageMultiplier() {
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        double multiplier = (.2 * voltage) - 1.5;
        return multiplier;
    }

    public void driveToPoint(double targetXInches, double heading, double speedModifier, double speedMinimum) {
        Robot.odometerRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.odometerLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.odometerRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.odometerLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        speedModifier = speedModifier * multiplier;
        double currentXInches;

        double startXPos = getAverageOdometerPosition();


        double lfPower;
        double rfPower;
        double lrPower;
        double rrPower;

        double currentSpeed;

        double maxWheelPower;
        double wheelPower = 0; //Minimum speed we start at
        double reverse = 1; // 1 is forward, -1 is backward


        ElapsedTime timeoutTimer = new ElapsedTime();

        currentXInches = (getAverageOdometerPosition() - startXPos);

        double distanceToX = targetXInches - currentXInches;

        currentSpeed = GetAverageVelocity();




        while ((Math.abs(distanceToX) > 0.25 || currentSpeed > .25) && opModeIsActive() /*&& timeoutTimer.seconds() < 1*/) {

            maxWheelPower = (Math.abs(Math.pow(distanceToX / speedModifier, 3)) + speedMinimum) / 100;

            double speedIncrease = .1;

            wheelPower += speedIncrease;
            if (Math.abs(wheelPower) > Math.abs(maxWheelPower)) {
                wheelPower = maxWheelPower;
            }

            double adjustment = headingAdjustment(heading);

            if (distanceToX < 0) {
                reverse = -1;
            } else {
                reverse = 1;
            }
            
            lfPower = (wheelPower * reverse + adjustment);
            rfPower = (wheelPower * reverse - adjustment);
            lrPower = (wheelPower * reverse + adjustment);
            rrPower = (wheelPower * reverse - adjustment);

            Robot.frontLeft.setPower(lfPower);
            Robot.frontRight.setPower(rfPower);
            Robot.backLeft.setPower(lrPower);
            Robot.backRight.setPower(rrPower);

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            currentXInches = (getAverageOdometerPosition() - startXPos);

            distanceToX = targetXInches - currentXInches;


            currentSpeed = GetAverageVelocity();

            if (Math.abs(currentSpeed) > .5) {
                timeoutTimer.reset();
            }


            telemetry.addData("XPos: ", currentXInches);
            telemetry.addData("distanceToX: ", distanceToX);
            telemetry.addData("Current Speed:", currentSpeed);
            telemetry.addData("Wheel Power: ", wheelPower);
            telemetry.addData("average: ", getAverageOdometerPosition());
            telemetry.addData("imu", angles.firstAngle);
            telemetry.update();

        }
        Robot.frontLeft.setPower(0);
        Robot.frontRight.setPower(0);
        Robot.backLeft.setPower(0);
        Robot.backRight.setPower(0);
    }

    public double getAverageOdometerPosition() {
        return ((Robot.odometerLeft.getCurrentPosition() + Robot.odometerRight.getCurrentPosition()) / 2.0) / Robot.odometerTicksPerInch;
    }
    public double GetAverageVelocity() {
        double averageVelocity = 0;
        averageVelocity = (Robot.odometerLeft.getVelocity() + Robot.odometerRight.getVelocity()) / 2;
        averageVelocity = (averageVelocity / Robot.odometerTicksPerInch) / 12;
        return averageVelocity;
    }
    public double headingAdjustment(double targetHeading) {
        double adjustment;
        double currentHeading;
        double degreesOff;
        boolean goRight;
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        currentHeading = angles.firstAngle;

        goRight = targetHeading > currentHeading;
        degreesOff = Math.abs(targetHeading - currentHeading);

        if (degreesOff > 180) {
            goRight = !goRight;
            degreesOff = 360 - degreesOff;
        }

        if (degreesOff < .3) {
            adjustment = 0;
        } else {
            adjustment = (Math.pow((degreesOff + 2) / 5, 2) + 2) / 100;
        }

        if (goRight) {
            adjustment = -adjustment;
        }
        return adjustment;
    }

}

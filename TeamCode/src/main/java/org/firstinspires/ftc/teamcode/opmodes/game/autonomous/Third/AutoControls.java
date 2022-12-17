package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.Third;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.robot.Robot;
import org.firstinspires.ftc.teamcode.hardware.robot.pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.hardware.robot.pipelines.PipelineColorCounting;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

public abstract class AutoControls extends LinearOpMode {


    Robot robot;
    BNO055IMU imu;
    Orientation angles;
    OpenCvWebcam webcam;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    //AprilTags
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 13, 14, 15 from 25h9 family
    int LEFT = 13;
    int MIDDLE = 14;
    int RIGHT = 15;

    AprilTagDetection tagOfInterest = null;

    static final double FEET_PER_METER = 3.28084;


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
    double turretPrevTargetDegrees = -1;
    double turretToleranceDegrees = 0.8;


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
    double liftHeightPrevTarget = -1;

    double liftToleranceInches = 0.25;
    //grabber
    double grabberServoClosedPos = Robot.grabberServoClosedPos;
    double grabberServoOpenPos = Robot.grabberServoOpenPos;
    double grabberServoHalfwayPos = Robot.grabberServoHalfwayPos;
    double grabberServoCurrentPos = 0;
    float changeFromZero = 0;

    double startingTicks;

    //turning
    double startingTurnSpeed = 0.4;
    double preciseTurnSpeed = 0.05;

    //strafe per inch
    double strafeTicksPerInch = 100.0 / (7.0/4.0);


    // hsvValues is an array that will hold the hue, saturation, and value information.
    float[] hsvValuesLeft = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float valuesLeft[] = hsvValuesLeft;

    float[] hsvValuesRight = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float valuesRight[] = hsvValuesRight;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;

    double multiplier = 0;
    public char alliance = 'b';

    public void init(HardwareMap hwMap) {
        Robot robot = new Robot(hwMap);
        initIMU();
        initCamera();
        telemetry.addData("here", "here");
        telemetry.update();
        sleep(1000);
        Robot.guideServo.setPosition(Robot.guideServoUp);
        Robot.grabberServo.setPosition(Robot.grabberServoOpenPos);

        ZeroPowerToBrake();
        multiplier = getVoltageMultiplier();

        //Camera Stuff
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

    public void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

    }
    public double getVoltageMultiplier() {
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        double multiplier = (.2 * voltage) - 1.5;
        return multiplier;
    }

    public int DetectAprilTags() {
        //April Tag Vision
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        webcam.setPipeline(aprilTagDetectionPipeline);

        telemetry.setMsTransmissionInterval(50);

        //Init loop INSTEAD OF waitForStart
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest");
                }

            } else {
                telemetry.addLine("Don't see tag of interest");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        if (tagOfInterest.id == LEFT) {
            return 1;
        } else if (tagOfInterest.id == MIDDLE) {
            return 2;
        } else {
            return 3;
        }
    }

    public void performAction(double targetXInches, double heading, double speedModifier, double speedMinimum, double liftHeightTarget, double liftPerformWithInchesLeft, double turretTargetDegrees, double turretPerformWithInchesLeft, double targetServoPosition, double servoPerformWithInchesLeft, double distanceToleranceParam) {

        speedModifier = speedModifier * 1; //multiplier;

        ResetEncoders();
        double currentLiftInches = 0;
        double liftInchesRemaining = 0;

        double currentTurretDegrees = 0;
        double turretDegreesRemaining = 0;
        double currentTurretTicks = 0;

        double currentXInches;

        double startXPos = getAverageOdometerPosition();


        double lfPower;
        double rfPower;
        double lrPower;
        double rrPower;

        double currentSpeed;
        double turningVelocity;

        double maxWheelPower;
        double wheelPower = 0; //Minimum speed we start at
        double reverse = 1; // 1 is forward, -1 is backward

        //Drive
        double distanceTolerance = 0.3; //inches away that allow us to exit the loop
        double stoppingSpeed = 0.25; //speed that is slow enough to exit the loop
        double turningVelocityTolerance = 0.3;

        if (distanceToleranceParam != 0) {
            distanceTolerance = distanceToleranceParam;
        }

        ElapsedTime timeoutTimer = new ElapsedTime();

        currentXInches = (getAverageOdometerPosition() - startXPos);

        double distanceToX = targetXInches - currentXInches;

        currentSpeed = GetAverageVelocity();
        turningVelocity = GetTurningVelocity();


        if (liftHeightTarget != -1) {
            currentLiftInches = Robot.liftMotor.getCurrentPosition() / liftTicksPerInch;
            liftInchesRemaining = Math.abs(liftHeightTarget - currentLiftInches);
        }

        if (turretTargetDegrees != -1) {
            currentTurretTicks = Robot.turretMotor.getCurrentPosition();
            currentTurretDegrees = currentTurretTicks / turretTicksPerDegree;
            turretDegreesRemaining = Math.abs(turretTargetDegrees - currentTurretDegrees);
        }


        while ((Math.abs(distanceToX) > distanceTolerance || currentSpeed > stoppingSpeed || turningVelocity > turningVelocityTolerance || liftInchesRemaining > liftToleranceInches || turretDegreesRemaining > turretToleranceDegrees || degreesOff(heading) > 1) && opModeIsActive() && timeoutTimer.milliseconds() < 500) {

            double adjustment = 0;
            if (heading != -1) {
                adjustment = headingAdjustment(heading, distanceToX);
            }

            if (Math.abs(distanceToX) > distanceTolerance || currentSpeed > stoppingSpeed) {
                maxWheelPower = (Math.abs(Math.pow(distanceToX / speedModifier, 3)) + speedMinimum) / 100;

                double speedIncrease = .1;

                wheelPower += speedIncrease;
                if (Math.abs(wheelPower) > Math.abs(maxWheelPower)) {
                    wheelPower = maxWheelPower;
                }

                if (distanceToX < 0) {
                    reverse = -1;
                } else {
                    reverse = 1;
                }
            }
            else {
                wheelPower = 0;
            }

            //double adjustForColorVariable = adjustForColor(alliance);

            lfPower = (wheelPower * reverse + adjustment);
            rfPower = (wheelPower * reverse - adjustment);
            lrPower = (wheelPower * reverse + adjustment);
            rrPower = (wheelPower * reverse - adjustment);

            Robot.frontLeft.setPower(lfPower);
            Robot.frontRight.setPower(rfPower);
            Robot.backLeft.setPower(lrPower);
            Robot.backRight.setPower(rrPower);


            currentXInches = (getAverageOdometerPosition() - startXPos);

            distanceToX = targetXInches - currentXInches;


            currentSpeed = GetAverageVelocity();

            if (Math.abs(currentSpeed) > .25) {
                timeoutTimer.reset();
            }


            if (turretTargetDegrees != -1 && Math.abs(distanceToX) <= turretPerformWithInchesLeft) {
                if (turretPrevTargetDegrees != turretTargetDegrees) {
                    TurnTurret(turretTargetDegrees);
                    telemetry.addData("turretMoving", "");
                    Robot.turretMotor.setPower(turretSpeed);
                    turretPrevTargetDegrees = turretTargetDegrees;
                }
            }

            if (liftHeightTarget != -1 && Math.abs(distanceToX) <= liftPerformWithInchesLeft) {
                if (liftHeightPrevTarget != liftHeightTarget) {
                    RaiseLift(liftHeightTarget);
                    Robot.liftMotor.setPower(liftSpeedUp);
                    telemetry.addData("liftMoving", "");
                    liftHeightPrevTarget = liftHeightTarget;
                }
            }
            if (targetServoPosition != -1 && Math.abs(distanceToX) <= servoPerformWithInchesLeft) {
                if (grabberServoCurrentPos != targetServoPosition) {
                    Robot.grabberServo.setPosition(targetServoPosition);
                    grabberServoCurrentPos = targetServoPosition;
                }
            }


            if (liftHeightTarget != -1) {
                currentLiftInches = Robot.liftMotor.getCurrentPosition() / liftTicksPerInch;
                liftInchesRemaining = Math.abs(liftHeightTarget - currentLiftInches);
            }

            if (turretTargetDegrees != -1) {
                currentTurretTicks = Robot.turretMotor.getCurrentPosition();
                currentTurretDegrees = currentTurretTicks / turretTicksPerDegree;
                turretDegreesRemaining = Math.abs(turretTargetDegrees - currentTurretDegrees);
            }


            telemetry.addData("XPos: ", currentXInches);
            telemetry.addData("distanceToX: ", distanceToX);
            telemetry.addData("Current Speed:", currentSpeed);
            telemetry.addData("Wheel Power: ", wheelPower);
            telemetry.addData("average: ", getAverageOdometerPosition());
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("imu", angles.firstAngle);
            telemetry.addData("Distance:" , Math.abs(distanceToX) > 0.25);
            telemetry.addData("Speed: ", currentSpeed > .25);
            telemetry.addData("Lift: ", liftInchesRemaining > liftToleranceInches);
            telemetry.addData("turret: ", turretDegreesRemaining > turretToleranceDegrees);
            telemetry.addData("rotation: ", degreesOff(heading) > 1);
            telemetry.update();

        }
        if (targetServoPosition != -1) {
            if (grabberServoCurrentPos != targetServoPosition) {
                Robot.grabberServo.setPosition(targetServoPosition);
                grabberServoCurrentPos = targetServoPosition;
            }
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
        double left = Robot.odometerLeft.getVelocity();
        double right =  Robot.odometerRight.getVelocity();
        averageVelocity = (left + right) / 2;
        averageVelocity = (averageVelocity / Robot.odometerTicksPerInch) / 12;
        return averageVelocity;
    }

    public double GetTurningVelocity() {
        double turningVelocity = 0;
        double left = Robot.odometerLeft.getVelocity();
        double right =  Robot.odometerRight.getVelocity();
        turningVelocity = Math.abs(right - left) / 2;
        return turningVelocity;
    }
    public double headingAdjustment(double targetHeading, double distanceToX) {
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

        double speedModifier = 14;
        double speedMinimum = 6;

        if (degreesOff > 10) {
            speedModifier = 10;
        }

        if (distanceToX > 3) {
            speedMinimum = 2;
        }


        if (degreesOff < .3) {
            adjustment = 0;
        } else {
            adjustment = (Math.pow((degreesOff + 2) / speedModifier, 2) + speedMinimum) / 100;
        }

        if (goRight) {
            adjustment = -adjustment;
        }
        return adjustment;
    }

    public double degreesOff(double targetHeading) {
        if (targetHeading == -1) {
            return 0;
        }
        double currentHeading;
        double degreesOff;

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        currentHeading = angles.firstAngle;

        degreesOff = Math.abs(targetHeading - currentHeading);

        if (degreesOff > 180) {

            degreesOff = 360 - degreesOff;
        }


        return degreesOff;
    }
    public double adjustForColor(char color) {
        double leftColor;
        double rightColor;

        double blueThreshold = 350;
        double blueDivisor = 4000;

        double redThreshold = 250;
        double redDivisor = 2000;

        double outputValue = 0;

        if (color == 'b') {
            leftColor = Robot.colorSensorLeft.red();
            rightColor = Robot.colorSensorRight.red();

            if (leftColor > redThreshold || rightColor > redThreshold) {
                outputValue = (rightColor - leftColor) * redDivisor;
            }
        }
        else {
            leftColor = Robot.colorSensorLeft.blue();
            rightColor = Robot.colorSensorRight.blue();

            if (leftColor > blueThreshold || rightColor > blueThreshold) {
                outputValue = (rightColor - leftColor) / blueDivisor;
            }
        }

        /*telemetry.addData("Left Color", leftColor);
        telemetry.addData("Right Color", rightColor);
        telemetry.addData("Output Value", outputValue);
        telemetry.update();*/

        return outputValue;


    }
    public void TurnTurret(double turretTargetDegrees) {

        Robot.turretMotor.setTargetPosition((int) (turretTargetDegrees * turretTicksPerDegree));
        Robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void RaiseLift(double liftHeightTarget) {

        Robot.liftMotor.setTargetPosition((int) (liftHeightTarget * liftTicksPerInch));
        Robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }




    public static void ResetEncoders() {
        Robot.odometerRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.odometerLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.odometerRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.odometerLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public static void StopEncoders() {
        Robot.odometerRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Robot.odometerLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static void ZeroPowerToBrake() {
        Robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

}

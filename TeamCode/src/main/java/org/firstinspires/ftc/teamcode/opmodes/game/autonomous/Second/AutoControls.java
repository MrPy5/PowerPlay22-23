package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.Second;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    double startingTicks;


    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValuesLeft[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float valuesLeft[] = hsvValuesLeft;

    float hsvValuesRight[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float valuesRight[] = hsvValuesRight;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;
    
    public void init(HardwareMap hwMap){
        Robot robot = new Robot(hwMap);
        initIMU();
        initCamera();
        //Camera Stuff
    }
    public void initIMU() {
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

        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);  // puts it in config mode
        sleep(100);
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0x0F);
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);

    }
    public void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {

                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {}
        });

    }
    public int DetectAprilTags() {
        //April Tag Vision
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        webcam.setPipeline(aprilTagDetectionPipeline);

        telemetry.setMsTransmissionInterval(50);

        //Init loop INSTEAD OF waitForStart
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest");
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        if (tagOfInterest.id == LEFT){
            return 1;
        } else if (tagOfInterest.id == MIDDLE) {
            return 2;
        } else {
            return 3;
        }
    }

    public void performAction(double driveInches, double liftHeightTarget, double liftPerformWithInchesLeft, double turretTargetDegrees, double turretPerformWithInchesLeft, char adjustForColor) {

        ResetEncoders();
        if (driveInches != 0) {
            Drive(driveInches);
        }

        if (liftHeightTarget != -1) {
            RaiseLift(liftHeightTarget);
        }

        if (turretTargetDegrees != -1) {
            TurnTurret(turretTargetDegrees);
        }

        boolean frontLeftBusy = driveInches != 0.0 && Robot.frontLeft.isBusy();
        boolean frontRightBusy = driveInches != 0.0 && Robot.frontRight.isBusy();
        boolean backLeftBusy = driveInches != 0.0 && Robot.backLeft.isBusy();
        boolean backRightBusy = driveInches != 0.0 && Robot.backRight.isBusy();

        boolean liftMotorBusy = liftHeightTarget != -1 && Robot.liftMotor.isBusy();
        boolean turretMotorBusy = turretTargetDegrees != -1 && Robot.turretMotor.isBusy();

        while (frontLeftBusy || frontRightBusy || backLeftBusy || backRightBusy || liftMotorBusy || turretMotorBusy) {

            frontLeftBusy = driveInches != 0.0 && Robot.frontLeft.isBusy();
            frontRightBusy = driveInches != 0.0 && Robot.frontRight.isBusy();
            backLeftBusy = driveInches != 0.0 && Robot.backLeft.isBusy();
            backRightBusy = driveInches != 0.0 && Robot.backRight.isBusy();

            liftMotorBusy = liftHeightTarget != -1 && Robot.liftMotor.isBusy();
            turretMotorBusy = turretTargetDegrees != -1 && Robot.turretMotor.isBusy();

            double currentFrontLeftTicks = Robot.frontLeft.getCurrentPosition();
            double driveInchesRemaining = driveInches - ((currentFrontLeftTicks) / countsPerInch);

            telemetry.addData("Inches Remaining", driveInchesRemaining);

            if (driveInches != 0.0) {
                if (driveInchesRemaining / driveInches >= 0.7) {
                    if (driveSpeedCurrent < driveSpeedFast) {
                        driveSpeedCurrent = driveSpeedCurrent + 0.01;
                    }
                }

                if (driveInchesRemaining / driveInches <= 0.3) {
                    /*if (driveSpeedCurrent > 0.3) {
                        driveSpeedCurrent = driveSpeedCurrent - 0.01;
                    } else {
                        driveSpeedCurrent = 0.3;
                    }*/
                    driveSpeedCurrent = 0.3;
                }
                Robot.frontLeft.setPower(driveSpeedCurrent);
                Robot.frontRight.setPower(driveSpeedCurrent);
                Robot.backLeft.setPower(driveSpeedCurrent);
                Robot.backRight.setPower(driveSpeedCurrent);
            }

            if (turretTargetDegrees != -1 && driveInchesRemaining < turretPerformWithInchesLeft) {
                telemetry.addData("turretMoving", "");

                Robot.turretMotor.setPower(turretSpeed);
            }

            if (liftHeightTarget != -1 && driveInchesRemaining < liftPerformWithInchesLeft) {
                Robot.liftMotor.setPower(liftSpeedUp);
                telemetry.addData("liftMoving", "");

            }
            telemetry.update();

        }
        Robot.frontLeft.setPower(0);
        Robot.frontRight.setPower(0);
        Robot.backLeft.setPower(0);
        Robot.backRight.setPower(0);
        if (adjustForColor == 'b') {
            StopEncoders();
            telemetry.addData("started sampling", "");

            Color.RGBToHSV((int) (Robot.colorSensorLeft.red() * SCALE_FACTOR),
                    (int) (Robot.colorSensorRight.green() * SCALE_FACTOR),
                    (int) (Robot.colorSensorLeft.blue() * SCALE_FACTOR),
                    hsvValuesLeft);

            Color.RGBToHSV((int) (Robot.colorSensorLeft.red() * SCALE_FACTOR),
                    (int) (Robot.colorSensorRight.green() * SCALE_FACTOR),
                    (int) (Robot.colorSensorRight.blue() * SCALE_FACTOR),
                    hsvValuesRight);

            telemetry.addData("Hue Left", hsvValuesLeft[0]);
            telemetry.addData("Hue Right", hsvValuesRight[0]);
            telemetry.update();
            sleep(5000);
            while (hsvValuesLeft[0] > hsvValuesRight[0] || hsvValuesRight[0] > hsvValuesLeft[0]) {

                Color.RGBToHSV((int) (Robot.colorSensorLeft.red() * SCALE_FACTOR),
                        (int) (Robot.colorSensorRight.green() * SCALE_FACTOR),
                        (int) (Robot.colorSensorLeft.blue() * SCALE_FACTOR),
                        hsvValuesLeft);

                Color.RGBToHSV((int) (Robot.colorSensorLeft.red() * SCALE_FACTOR),
                        (int) (Robot.colorSensorRight.green() * SCALE_FACTOR),
                        (int) (Robot.colorSensorRight.blue() * SCALE_FACTOR),
                        hsvValuesRight);

                if (hsvValuesLeft[0] < hsvValuesRight[0]) {
                    //MoveLeft
                    Robot.frontLeft.setPower(-0.15);
                    Robot.frontRight.setPower(0.15);
                    Robot.backLeft.setPower(0.15);
                    Robot.backRight.setPower(-0.15);

                }

                if (hsvValuesRight[0] < hsvValuesLeft[0]) {
                    //MoveRight
                    Robot.frontLeft.setPower(0.1);
                    Robot.frontRight.setPower(-0.1);
                    Robot.backLeft.setPower(-0.1);
                    Robot.backRight.setPower(0.1);
                }


                /*if (Math.abs(hsvValuesRight[0]  - hsvValuesLeft[0]) < 10) {
                    Robot.frontLeft.setPower(0);
                    Robot.frontRight.setPower(-0);
                    Robot.backLeft.setPower(-0);
                    Robot.backRight.setPower(0);
                    break;
                }*/
                telemetry.addData("Hue Left", hsvValuesLeft[0]);
                telemetry.addData("Hue Right", hsvValuesRight[0]);
                telemetry.update();
            }



        }
        Robot.liftMotor.setPower(0);
        Robot.turretMotor.setPower(0);

        Robot.frontLeft.setPower(0);
        Robot.frontRight.setPower(0);
        Robot.backLeft.setPower(0);
        Robot.backRight.setPower(0);


    }
    /*public void BatchUpdate(boolean drive, double batchDriveTarget, boolean lift, double batchLiftTarget, boolean turret, double batchTurretTarget) {
        if (drive) {
            Drive((int) batchDriveTarget);
        }

        if (lift) {
            RaiseLift((int) batchLiftTarget);
        }

        if (turret) {
            TurnTurret((int) batchTurretTarget);
        }

        boolean frontLeftBusy = false;
        boolean frontRightBusy = false;
        boolean backLeftBusy = false;
        boolean backRightBusy = false;
        boolean liftMotorBusy = false;
        boolean turretMotorBusy = false;
        if (drive) {
            frontLeftBusy = Robot.frontLeft.isBusy();
            frontRightBusy = Robot.frontRight.isBusy();
            backLeftBusy = Robot.backLeft.isBusy();
            backRightBusy = Robot.backRight.isBusy();
        }
        else {
            frontLeftBusy = false;
            frontRightBusy = false;
            backLeftBusy = false;
            backRightBusy = false;
        }
        if (lift) {
            liftMotorBusy = Robot.liftMotor.isBusy();
        }
        else {
            liftMotorBusy = false;
        }

        if (turret) {
            turretMotorBusy = Robot.turretMotor.isBusy();
        }
        else {
            turretMotorBusy = false;
        }
        while ((frontLeftBusy || frontRightBusy || backLeftBusy || backRightBusy|| liftMotorBusy || turretMotorBusy)) {
                /*telemetry.addData("frontLeft:", Robot.frontLeft.isBusy());
                telemetry.addData("frontRight:", Robot.frontRight.isBusy());
                telemetry.addData("backLeft:", Robot.backLeft.isBusy());
                telemetry.addData("backRight:", Robot.backRight.isBusy());
                telemetry.addData("liftMotor:", Robot.liftMotor.isBusy());
                telemetry.addData("turretMotor:", Robot.turretMotor.isBusy());
                telemetry.update();*/

            /*if (drive) {
                frontLeftBusy = Robot.frontLeft.isBusy();
                frontRightBusy = Robot.frontRight.isBusy();
                backLeftBusy = Robot.backLeft.isBusy();
                backRightBusy = Robot.backRight.isBusy();
            }
            else {
                frontLeftBusy = false;
                frontRightBusy = false;
                backLeftBusy = false;
                backRightBusy = false;
            }
            if (lift) {
                liftMotorBusy = Robot.liftMotor.isBusy();
            }
            else {
                liftMotorBusy = false;
            }

            if (turret) {
                turretMotorBusy = Robot.turretMotor.isBusy();
            }
            else {
                turretMotorBusy = false;
            }

            if (Robot.frontLeft.getCurrentPosition() / ((int) batchDriveTarget * countsPerInch) <= 0.3) {
                if (driveSpeedCurrent < driveSpeedFast) {
                    driveSpeedCurrent = driveSpeedCurrent + 0.01;
                }
            }
            if (drive) {
                if (Robot.frontLeft.getCurrentPosition() / ((int) batchDriveTarget * countsPerInch) >= 0.7) {
                    if (driveSpeedCurrent > 0.3) {
                        driveSpeedCurrent = driveSpeedCurrent - 0.01;
                    } else {
                        driveSpeedCurrent = 0.3;
                    }
                }
                Robot.frontLeft.setPower(driveSpeedCurrent);
                Robot.frontRight.setPower(driveSpeedCurrent);
                Robot.backLeft.setPower(driveSpeedCurrent);
                Robot.backRight.setPower(driveSpeedCurrent);
            }
        }
        Robot.liftMotor.setPower(0);

        Robot.turretMotor.setPower(0);
        Robot.frontLeft.setPower(0);
        Robot.frontRight.setPower(0);
        Robot.backLeft.setPower(0);
        Robot.backRight.setPower(0);
    }*/


    public void Drive(double inches) {

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
        /*telemetry.addData(">", currentAngle - targetAngle);
        telemetry.update();*/
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
            /*telemetry.addData("> ", currentAngle);
            telemetry.addData("> ", angles.firstAngle - changeFromZero);
            telemetry.update();*/

        }

        Robot.frontLeft.setPower(0);
        Robot.frontRight.setPower(0);
        Robot.backLeft.setPower(0);
        Robot.backRight.setPower(0);
    }


    public void TurnTurret(double turretTargetDegrees) {
        liftCurrentHeight = Robot.liftMotor.getCurrentPosition() / liftTicksPerInch;
        turretCurrentDegrees = Robot.turretMotor.getCurrentPosition() / turretTicksPerDegree;

        Robot.turretMotor.setTargetPosition((int) (turretTargetDegrees * turretTicksPerDegree));
        Robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);



    }

    public void RaiseLift(double liftHeightTarget) {
        liftCurrentHeight = Robot.liftMotor.getCurrentPosition() / liftTicksPerInch;
        turretCurrentDegrees = Robot.turretMotor.getCurrentPosition() / turretTicksPerDegree;

        Robot.liftMotor.setTargetPosition((int) (liftHeightTarget * liftTicksPerInch));
        Robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }
    public void ChangeGripperState(double gripperTarget) {

        Robot.grabberServo.setPosition(gripperTarget);
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
    
    public static void StopEncoders() {

        Robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

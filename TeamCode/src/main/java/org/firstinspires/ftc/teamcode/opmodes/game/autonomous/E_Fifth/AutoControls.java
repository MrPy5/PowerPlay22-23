package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.E_Fifth;

import android.annotation.SuppressLint;
import android.util.Log;

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
import org.firstinspires.ftc.teamcode.opmodes.testing.AdjustmentConstants;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

public abstract class AutoControls extends LinearOpMode {

    boolean log = false;
    boolean doTelemetry = true;

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
    static double turretSpeed = 0.6;
    double turretCurrentDegrees;
    double turretCloseToZero = 70;

    static double turretTicksPerDegree = Robot.turretTicksPerDegree;

    public static double turretForwardDegrees = 0; //all rotation variables in degrees
    public static double turretRightDegrees = 90;
    public static double turretLeftDegrees = -90;
    public static double turretBackDegrees = 180;

    public static double turretTargetDegrees = 0;
    static double turretPrevTargetDegrees = -1;
    static double turretToleranceDegrees = 0.8;


    //lift
    static double liftSpeedUp = 1;
    double liftSpeedDown = .5;
    double liftSpeedPower;

    static double liftTicksPerInch = Robot.liftTicksPerInch;

    double liftCurrentHeight; //all height variables are in inches
    double liftPickupHeight = 0;
    double liftJunctionGroundHeight = 2;
    double liftJunctionLowHeight = 15;
    double liftJunctionMediumHeight = 24;
    double liftJunctionHighHeight = 34;
    double liftMinHeightForTurning = 6;
    double liftMaximumHeight = 36;

    double liftHeightTarget = 0;
    static double liftHeightPrevTarget = -1;

    static double liftToleranceInches = 0.25;
    //grabber
    double grabberServoClosedPos = Robot.grabberServoClosedPos;
    double grabberServoOpenPos = Robot.grabberServoOpenPos;
    double grabberServoHalfwayPos = Robot.grabberServoHalfwayPos;
    static double grabberServoCurrentPos = 0;
    float changeFromZero = 0;

    double startingTicks;

    //turning
    double startingTurnSpeed = 0.4;
    double preciseTurnSpeed = 0.05;

    //strafe per inch
    double strafeTicksPerInch = 100.0 / (7.0/4.0);


    // hsvValues is an array that will hold the hue, saturation, and value information.
    static float[] hsvValuesLeft = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    static final float valuesLeft[] = hsvValuesLeft;

    static float[] hsvValuesRight = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    static final float valuesRight[] = hsvValuesRight;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    static final double SCALE_FACTOR = 255;

    static double multiplier = 0;
    static public char alliance = 'b';
    static public char side = 'l';

    static double coneDifference = 1.3125;
    static double coneOneGrabHeight = 6.4;
    static double coneTwoGrabHeight = coneOneGrabHeight - coneDifference;
    static double coneThreeGrabHeight = coneTwoGrabHeight - coneDifference;
    static double coneFourGrabHeight = coneThreeGrabHeight - coneDifference;
    static double coneFiveGrabHeight = coneFourGrabHeight - coneDifference;

    static ElapsedTime gameTimer = new ElapsedTime();

    static ElapsedTime cUMoveTimer = new ElapsedTime();

    double cULastMoveTime = 0;
    double cuLeftPos = Robot.cULeftClosedPos;
    double cuRightPos = Robot.cURightClosedPos;

    double cuMilliseconds = 3;

    static double quitTime = 29850;

    public void init(HardwareMap hwMap) {
        Robot robot = new Robot(hwMap, false);
        initIMU();
        initCamera();
        telemetry.addData("here", "here");
        telemetry.update();
        Robot.guideServo.setPosition(Robot.guideServoUp);
        //Robot.grabberServo.setPosition(Robot.grabberServoOpenPos + 0.15);
        Robot.coneUprightRightServo.setPosition(Robot.cURightClosedPos);
        Robot.coneUprightLeftServo.setPosition(Robot.cULeftClosedPos);

        ZeroPowerToBrake();
        multiplier = getVoltageMultiplier();
        cUMoveTimer.startTime();

        Robot.turretMotor.setTargetPosition(0);
        Robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Robot.turretMotor.setPower(Robot.turretSpeed);

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
        double multiplier = 1;
        if (voltage > 12) {
            multiplier = 1 + ((voltage - 13) / 13);
        }

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
        gameTimer.reset();
        cUMoveTimer.reset();
        if (tagOfInterest.id == LEFT) {
            return 1;
        } else if (tagOfInterest.id == MIDDLE) {
            return 2;
        } else {
            return 3;
        }

    }

    public void performAction(double targetXInches, double heading, double speedModifier, double speedMinimum, double liftHeightTarget, double liftPerformWithInchesLeft, double turretTargetDegrees, double turretPerformWithInchesLeft, double targetServoPosition, double servoPerformWithInchesLeft, double distanceToleranceParam, double liftQuitWithInchesLeft, boolean colorCorrection, double[] cuInfo) {

        //speedModifier = speedModifier * multiplier; //multiplier;
        if (log) {
            Log.d("performActionBegin","targetXInches:" + targetXInches
                    + " heading:" + heading
                    + " speedModifier:" + speedModifier
                    + " speedMinimum:" + speedMinimum
                    + " liftHeightTarget:" + liftHeightTarget
                    + " liftPerformWithInchesLeft:" + liftPerformWithInchesLeft
                    + " turretTargetDegrees:" + turretTargetDegrees
                    + " turretPerformWithInchesLeft:" + turretPerformWithInchesLeft
                    + " targetServoPosition:" + targetServoPosition
                    + " servoPerformWithInchesLeft:" + servoPerformWithInchesLeft
                    + " distanceToleranceParam:" + distanceToleranceParam
                    + " liftQuitWithInchesLeft:" + liftQuitWithInchesLeft
                    + " colorCorrection:" + colorCorrection
                    + " cuInfo:" + cuInfo);

        }

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
        double stoppingSpeed = 0.4; //speed that is slow enough to exit the loop
        double turningVelocityTolerance = 0.3;

        if (distanceToleranceParam != 0) {
            distanceTolerance = distanceToleranceParam;
        }

        /*
        if (servoPerformWithInchesLeft <= distanceTolerance) {
            servoPerformWithInchesLeft = distanceTolerance + 0.01;
        }
         */


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

        //---First turn---//
        while (degreesOff(heading) > .5 && opModeIsActive() && gameTimer.milliseconds() < quitTime) {
            double adjustment = 0;
            if (heading != -1) {
                adjustment = headingAdjustment(heading, 0);
            }

            lfPower = adjustment;
            rfPower = -adjustment;
            lrPower = adjustment;
            rrPower = -adjustment;

            Robot.frontLeft.setPower(lfPower / multiplier);
            Robot.frontRight.setPower(rfPower / multiplier);
            Robot.backLeft.setPower(lrPower / multiplier);
            Robot.backRight.setPower(rrPower / multiplier);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if (Math.abs(angles.secondAngle) > 10 || Math.abs(angles.thirdAngle) > 10) {
                requestOpModeStop();
            }


        }


        ElapsedTime timeoutTimer = new ElapsedTime();

        //---Main Loop---//
        while ((Math.abs(distanceToX) > distanceTolerance || Math.abs(currentSpeed) > stoppingSpeed /*|| turningVelocity > turningVelocityTolerance */
                || liftInchesRemaining > liftToleranceInches || turretDegreesRemaining > turretToleranceDegrees /*|| degreesOff(heading) > 1 */)
                && opModeIsActive()
                && timeoutTimer.milliseconds() < 500
                && (liftHeightTarget == -1 || liftInchesRemaining > liftQuitWithInchesLeft)
                && gameTimer.milliseconds() < quitTime) {

            double adjustment = 0;
            if (heading != -1/* && distanceToX > 3*/) {
                adjustment = headingAdjustment(heading, distanceToX);
            }

            if (Math.abs(distanceToX) > distanceTolerance || Math.abs(currentSpeed) > stoppingSpeed) {
                maxWheelPower = (Math.abs(Math.pow((distanceToX) / speedModifier, 3.0)) + speedMinimum) / 100;

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

            double adjustForColorVariable = 0;
            if (colorCorrection && Math.abs(distanceToX) <= 17) {
                adjustForColorVariable = adjustForColorPlusWander(alliance, currentSpeed);
            }

            lfPower = (wheelPower * reverse) + adjustment + adjustForColorVariable;
            rfPower = (wheelPower * reverse) - adjustment - adjustForColorVariable;
            lrPower = (wheelPower * reverse) + adjustment - adjustForColorVariable;
            rrPower = (wheelPower * reverse) - adjustment + adjustForColorVariable;

            Robot.frontLeft.setPower(lfPower / multiplier);
            Robot.frontRight.setPower(rfPower / multiplier);
            Robot.backLeft.setPower(lrPower / multiplier);
            Robot.backRight.setPower(rrPower / multiplier);


            currentXInches = (getAverageOdometerPosition() - startXPos);

            distanceToX = targetXInches - currentXInches;


            currentSpeed = GetAverageVelocity();

            if (Math.abs(currentSpeed) > stoppingSpeed - 0.05) {
                timeoutTimer.reset();
            }


            liftCurrentHeight = Robot.liftMotor.getCurrentPosition() / Robot.liftTicksPerInch;

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
                    double liftSpeedPower = 0;
                    if (liftCurrentHeight < liftHeightTarget) {
                        liftSpeedPower = 1;
                    }
                    else {
                        liftSpeedPower = 0.6;
                    }
                    Robot.liftMotor.setPower(liftSpeedPower);
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

            /*if (currentLiftInches > 3 && cuInfo[0] != -1) {
                if (cuInfo[1] == 0) {
                    Robot.coneUprightLeftServo.setPosition(Robot.cULeftSweepPos);
                    cuLeftPos = Robot.cULeftSweepPos;
                }

                if (cuInfo[1] == 1) {
                    Robot.coneUprightRightServo.setPosition(Robot.cURightSweepPos);
                    cuRightPos = Robot.cURightSweepPos;
                }
            }*/
            if (Math.abs(distanceToX) <= cuInfo[0] && cuInfo[0] != -1) {
                if (cuInfo[1] == 0) {

                    /*if (cUMoveTimer.milliseconds() > cULastMoveTime + cuMilliseconds) {
                            cULastMoveTime = cUMoveTimer.milliseconds();


                            if (cuLeftPos != cuInfo[2]) {
                                if (cuLeftPos < cuInfo[2]) {
                                    cuLeftPos += 0.010;
                                }
                                if (cuLeftPos > cuInfo[2]) {
                                    cuLeftPos -= 0.010;
                                }
                            }


                    }*/
                    Robot.coneUprightLeftServo.setPosition(cuInfo[2]);


                }
                if (cuInfo[1] == 1) {
                    /*if (cUMoveTimer.milliseconds() > cULastMoveTime + cuMilliseconds) {
                        cULastMoveTime = cUMoveTimer.milliseconds();


                        if (cuRightPos != cuInfo[2]) {
                            if (cuRightPos < cuInfo[2]) {
                                cuRightPos +=0.010;
                            }
                            if (cuRightPos > cuInfo[2]) {
                                cuRightPos -= 0.010;
                            }
                        }


                    }*/
                    Robot.coneUprightRightServo.setPosition(cuInfo[2]);

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

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if (doTelemetry) {
                telemetry.addData("color: ", adjustForColorVariable);

                telemetry.addData("imu", angles.firstAngle);
                telemetry.addData("headingAdjustment: ", adjustment);
                telemetry.addData("game time", cUMoveTimer.seconds());
                telemetry.addData("XPos: ", currentXInches);
                telemetry.addData("distanceToX: ", distanceToX);
                telemetry.addData("Current Speed:", currentSpeed);
                telemetry.addData("Wheel Power: ", wheelPower);
                telemetry.addData("Turning Velocity: ", turningVelocity);
                telemetry.addData("average: ", getAverageOdometerPosition());
                telemetry.addData("Distance:", Math.abs(distanceToX) > distanceTolerance);
                telemetry.addData("Speed: ", currentSpeed > .25);
                telemetry.addData("Lift: ", liftInchesRemaining > liftToleranceInches);
                telemetry.addData("turret: ", turretDegreesRemaining > turretToleranceDegrees);
                telemetry.addData("rotation: ", degreesOff(heading) > 1);
                telemetry.addData("Turning Velocity: ", turningVelocity > turningVelocityTolerance);
                telemetry.addData("Lift done: ", liftInchesRemaining > liftQuitWithInchesLeft);
                telemetry.update();
            }
            if (Math.abs(angles.secondAngle) > 10 || Math.abs(angles.thirdAngle) > 10) {
                requestOpModeStop();
            }

            if (log) {
                Log.d("performActionLoop",
                        "DistanceToX:" + distanceToX
                        + " liftInchesRemaining: " + liftInchesRemaining
                        + " firstAngle: " + angles.firstAngle
                        + " game time: " + gameTimer.milliseconds()
                        + " adjustmentforAngle: " + adjustment
                        + " adjustmentforColor: " + adjustForColorVariable
                        + " rightFront:" + rfPower
                        + " rightRear:" + rrPower
                        + " leftFront:" + lfPower
                        + " leftRear:" + lrPower
                        + " reverse: " + reverse);
            }

        }

        Robot.frontLeft.setPower(0);
        Robot.frontRight.setPower(0);
        Robot.backLeft.setPower(0);
        Robot.backRight.setPower(0);

        if (targetServoPosition != -1) {
            Robot.grabberServo.setPosition(targetServoPosition);
            grabberServoCurrentPos = targetServoPosition;
        }

        /*if (Math.abs(distanceToX) <= cuInfo[0] && cuInfo[0] != -1) {
            if (cuInfo[1] == 0) {
                while (!(cuLeftPos + 0.01 > cuInfo[2] && cuLeftPos - 0.01 < cuInfo[2]) && opModeIsActive() && gameTimer.milliseconds() < quitTime) {
                    telemetry.addData("cu", cuLeftPos);
                    telemetry.update();
                    if (cUMoveTimer.milliseconds() > cULastMoveTime + cuMilliseconds) {
                        cULastMoveTime = cUMoveTimer.milliseconds();


                        if (cuLeftPos != cuInfo[2]) {
                            if (cuLeftPos < cuInfo[2]) {
                                cuLeftPos += 0.010;
                            }
                            if (cuLeftPos > cuInfo[2]) {
                                cuLeftPos -= 0.010;
                            }
                        }


                    }
                    Robot.coneUprightLeftServo.setPosition(cuLeftPos);
                }


            }
            if (cuInfo[1] == 1) {
                while (!(cuRightPos + 0.01 > cuInfo[2] && cuRightPos - 0.01 < cuInfo[2]) && opModeIsActive() && gameTimer.milliseconds() < quitTime) {
                    if (cUMoveTimer.milliseconds() > cULastMoveTime + cuMilliseconds) {
                        cULastMoveTime = cUMoveTimer.milliseconds();


                        if (cuRightPos != cuInfo[2]) {
                            if (cuRightPos < cuInfo[2]) {
                                cuRightPos += 0.010;
                            }
                            if (cuRightPos > cuInfo[2]) {
                                cuRightPos -= 0.010;
                            }
                        }


                    }
                    Robot.coneUprightRightServo.setPosition(cuRightPos);
                }

            }
        }*/



        /*if (colorCorrection) {
            double adjustForColorVariable = 0;

            adjustForColorVariable = adjustForColorPlusWander(alliance);
            while (adjustForColorVariable > 0.08) {
                adjustForColorVariable = adjustForColorPlusWander(alliance);

                lfPower = (adjustForColorVariable);
                rfPower = (-adjustForColorVariable);
                lrPower = (-adjustForColorVariable);
                rrPower = (adjustForColorVariable);

                Robot.frontLeft.setPower(lfPower / multiplier);
                Robot.frontRight.setPower(rfPower / multiplier);
                Robot.backLeft.setPower(lrPower / multiplier);
                Robot.backRight.setPower(rrPower / multiplier);
            }
        }*/

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

    public double GetAverageVelocityMecanum() {
        double averageVelocity;

        double backLeft = Robot.backLeft.getVelocity();
        double backRight = Robot.backRight.getVelocity();
        double frontRight = Robot.frontRight.getVelocity();
        double frontLeft = Robot.frontLeft.getVelocity();

        averageVelocity = (backLeft + backRight + frontRight + frontLeft) / 4;
        averageVelocity = (averageVelocity / Robot.ticksPerInch) / 12;
        return  averageVelocity;
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

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        currentHeading = (360 + angles.firstAngle) % 360;

        goRight = targetHeading > currentHeading;
        degreesOff = Math.abs(targetHeading - currentHeading);

        if (degreesOff > 180) {
            goRight = !goRight;
            degreesOff = 360 - degreesOff;
        }

        double speedMinimum;
        double speedModifier = AdjustmentConstants.speedModifier;

        if (degreesOff > 10) {
            speedModifier = 10;
        }

        if (distanceToX == 0) {  // this????
            speedMinimum = AdjustmentConstants.speedMinimum;
        } else {
            speedMinimum = 3;
        }

        if (degreesOff < .3) {
            adjustment = 0;
        } else {
            adjustment = (Math.pow((degreesOff + AdjustmentConstants.graphShift) / speedModifier, AdjustmentConstants.curvePower) + speedMinimum) / 100;
        }

        if (goRight) {
            adjustment = -adjustment;
        }
        if (log) {
            Log.d("headingAdjustment", "currentHeading:" + currentHeading +
                    " targetHeading:" + targetHeading +
                    " Adjustment:" + adjustment +
                    " DistanceToX:" + distanceToX +
                    " SpeedModifier:" + AdjustmentConstants.speedModifier +
                    " SpeedMinimum:" + AdjustmentConstants.speedMinimum +
                    " Degrees Off:" + degreesOff);
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

        currentHeading = (360 + angles.firstAngle) % 360;

        degreesOff = Math.abs(targetHeading - currentHeading);

        if (degreesOff > 180) {

            degreesOff = 360 - degreesOff;
        }

        if (log) {
            Log.d("degreesOff", "degreesOff:" + degreesOff +
                    " currentHeading:" + currentHeading +
                    " targetHeading:" + targetHeading);
        }
        return degreesOff;
    }

    public double adjustForColorPlusWander(char color, double currentSpeed) {
        double leftColor;
        double rightColor;

        double blueThreshold = 290;
        double blueDivisor = 3500;
        if (currentSpeed < 1.5) {
            blueDivisor = 7000;
        }
        else {
            blueDivisor = 2000;
        }
        double redThreshold = 250;
        double redDivisor = 4500;
        if (currentSpeed < 1.5) {
            redDivisor = 4000;
        }
        else {
            redDivisor = 3500;
        }

        double outputValue = 0;

        if (color == 'r') {
            leftColor = Robot.colorSensorLeft.red();
            rightColor = Robot.colorSensorRight.red();

            if (leftColor > redThreshold || rightColor > redThreshold) {
                outputValue = (rightColor - leftColor) / redDivisor;
            }
            /*else {
                if (side == 'l') {
                    outputValue = 0.2;
                }
                else {
                    outputValue = -0.2;
                }
            }*/
        }
        else {
            leftColor = Robot.colorSensorLeft.blue();
            rightColor = Robot.colorSensorRight.blue();

            if (leftColor > blueThreshold || rightColor > blueThreshold) {
                outputValue = (rightColor - leftColor) / blueDivisor;
            }
            /*else {
                if (side == 'l') {
                    outputValue = 0.2;
                }
                else {
                    outputValue = -0.2;
                }
            }*/
        }
        /*
        if (doTelemetry) {
            telemetry.addData("Left Color", leftColor);
            telemetry.addData("Right Color", rightColor);
            telemetry.addData("Output Value", outputValue);
            telemetry.update();
        }*/
        /*if (log)*/ {
            Log.d("adjustForColorPlusWander", "color:" + color +
                    " outputValue:" + outputValue +
                    " rightColor:" + rightColor +
                    " leftColor:" + leftColor);
        }

        return outputValue;


    }

    public double adjustForColor(char color) {
        double leftColor;
        double rightColor;

        double blueThreshold = 350;
        double blueDivisor = 5250;

        double redThreshold = 200;
        double redDivisor = 5500;

        double outputValue = 0;

        if (color == 'r') {
            leftColor = Robot.colorSensorLeft.red();
            rightColor = Robot.colorSensorRight.red();

            if (leftColor > redThreshold || rightColor > redThreshold) {
                outputValue = (rightColor - leftColor) / redDivisor;
            }
        }
        else {
            leftColor = Robot.colorSensorLeft.blue();
            rightColor = Robot.colorSensorRight.blue();

            if (leftColor > blueThreshold || rightColor > blueThreshold) {
                outputValue = (rightColor - leftColor) / blueDivisor;
            }

        }
        if (doTelemetry) {
            telemetry.addData("Left Color", leftColor);
            telemetry.addData("Right Color", rightColor);
            telemetry.addData("Output Value", outputValue);
            telemetry.update();
        }
        if (log) {
            Log.d("adjustForColor", "color:" + color +
                    " outputValue:" + outputValue +
                    " rightColor:" + rightColor +
                    " leftColor:" + leftColor);
        }

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
    public static void ZeroPowerToFloat() {
        Robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection)
    {
        if (doTelemetry) {
            telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
            telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
            telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
            telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
            telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
            telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
            telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        }
    }



}

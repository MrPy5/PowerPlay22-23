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

package org.firstinspires.ftc.teamcode.opmodes.game.autonomous.Second;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
import java.util.Locale;


@Autonomous(name="L Blue GAME AUTO CALM AT")
@Disabled
public class AutoCalmLeftBlueAT extends LinearOpMode {

    /* Declare OpMode members. */
    OpenCvWebcam webcam;
    BNO055IMU imu;
    Orientation angles;
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

    //drive
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


    //batchUpdate;

    public double firstDriveTarget;
    public double batchLiftTarget;

    //
    public int endParkingPosition = 1;
    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap);
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



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        webcam.setPipeline(new PipelineColorCounting(640, 480));

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
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
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

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

        waitForStart();

        if (tagOfInterest.id == LEFT){
            endParkingPosition = 1;
        } else if (tagOfInterest.id == MIDDLE) {
            endParkingPosition = 2;
        } else if (tagOfInterest.id == RIGHT){
            endParkingPosition = 3;
        }

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //float currentHeading = Float.parseFloat(formatAngle(angles.angleUnit, angles.firstAngle));
        changeFromZero = angles.firstAngle;

        telemetry.addData("Tag : Park - ", tagOfInterest.id + " : " + endParkingPosition);

        telemetry.update();

        //Close servo to start match
        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        sleep(500);

        ZeroPowerToBrake();

        //Get to High cone and move turret Right
        firstDriveTarget = 63.5;
        batchLiftTarget = Robot.liftJunctionHighHeight;

        BatchUpdate(true, firstDriveTarget, true, batchLiftTarget, true, turretRightDegrees);


        //Drop turret
        BatchUpdate(false, 0, true, Robot.liftJunctionHighHeight - 2, false, 0);
        sleep(200);

        //Release servo
        Robot.grabberServo.setPosition(Robot.grabberServoOpenPos);
        sleep(500);

        //Raise turret to original height
        BatchUpdate(false, 0, true, Robot.liftJunctionHighHeight, false, 0);

        //Reverse to 5 stack, forward, drop to one
        BatchUpdate(false, 0, false, 0, true, Robot.turretForwardDegrees);
        BatchUpdate(true, -10, false, 0, false, 0);

        //Turn to face 5 stack
        Turn(90);

        //Lower Drive Speed
        driveSpeedFast = 0.3;

        // Forward to 5 stack, raise to pickup height
        BatchUpdate(true, 23.5, true, 6.25, false, 0);

        //Close servo
        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);

        //Wait
        sleep(500);

        //Raise lift a bit
        BatchUpdate(false, 0, true, Robot.liftPickupHeight + 10, false, 0);
        //Raise lift rest of the way, back to high junction
        BatchUpdate(true, -34, true, Robot.liftJunctionHighHeight, true, turretRightDegrees);

        BatchUpdate(false, 0, true, Robot.liftJunctionHighHeight - 2, false, 0);

        sleep(200);
        //open servo
        Robot.grabberServo.setPosition(Robot.grabberServoOpenPos);
        //turn forward
        sleep(350);

        /*BatchUpdate(false, 0, true, Robot.liftJunctionHighHeight,  false, 0);

        BatchUpdate(false, 0, false, 0, true, turretForwardDegrees);

        //Forward to 5 stack, lift height
        BatchUpdate(true, 37, true, Robot.liftPickupHeight + 4.5, false, 0);
        //Close servo
        Robot.grabberServo.setPosition(Robot.grabberServoClosedPos);
        //Wait
        sleep(300);
        //10 height
        BatchUpdate(false, 0, true, Robot.liftPickupHeight + 10, false, 0);
        //Rest of the way, back to high junction
        BatchUpdate(true, -37, true, Robot.liftJunctionHighHeight, true, turretRightDegrees);
        BatchUpdate(false, 0, true, Robot.liftJunctionHighHeight - 2, false, 0);

        //open servo
        Robot.grabberServo.setPosition(Robot.grabberServoOpenPos);
        //turn forward
        sleep(300);*/

        BatchUpdate(false, 0, true, Robot.liftJunctionHighHeight,  false, 0);

        if (endParkingPosition == 3) {
            BatchUpdate(true, -13, false, 0,false, 0);
        }

        else if (endParkingPosition == 2) {
            BatchUpdate(true, 14, false, 0,false, 0);
        }

        else if (endParkingPosition == 1) {
            telemetry.addData("LEFT!!!!", "");
            telemetry.update();
            BatchUpdate(true, 33, false, 0,false, 0);


        }

        else {
            BatchUpdate(true, 14, false, 0,false, 0);
        }

        Turn(180);
        BatchUpdate(false, 0, false, 0, true, turretForwardDegrees);
        BatchUpdate(false, 0, true, 0, false, 0);

    }

    public void BatchUpdate(boolean drive, double batchDriveTarget, boolean lift, double batchLiftTarget, boolean turret, double batchTurretTarget) {


        if (drive) {
            Drive(batchDriveTarget);
        }

        if (lift) {
            RaiseLift(batchLiftTarget);
        }

        if (turret) {
            TurnTurret(batchTurretTarget);
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
        if (lift) {
            liftMotorBusy = Robot.liftMotor.isBusy();
        }

        if (turret) {
            turretMotorBusy = Robot.turretMotor.isBusy();
        }

        while ((frontLeftBusy || frontRightBusy || backLeftBusy || backRightBusy|| liftMotorBusy || turretMotorBusy)) {
                /*telemetry.addData("frontLeft:", Robot.frontLeft.isBusy());
                telemetry.addData("frontRight:", Robot.frontRight.isBusy());
                telemetry.addData("backLeft:", Robot.backLeft.isBusy());
                telemetry.addData("backRight:", Robot.backRight.isBusy());
                telemetry.addData("liftMotor:", Robot.liftMotor.isBusy());
                telemetry.addData("turretMotor:", Robot.turretMotor.isBusy());
                telemetry.update();*/

            if (drive) {
                frontLeftBusy = Robot.frontLeft.isBusy();
                frontRightBusy = Robot.frontRight.isBusy();
                backLeftBusy = Robot.backLeft.isBusy();
                backRightBusy = Robot.backRight.isBusy();
            }

            if (lift) {
                liftMotorBusy = Robot.liftMotor.isBusy();
            }


            if (turret) {
                turretMotorBusy = Robot.turretMotor.isBusy();
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
    }


    public void Drive(double inches) {
        ResetEncoders();
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
        Robot.turretMotor.setPower(turretSpeed);
        turretPrevTargetDegrees = turretTargetDegrees;





    }

    public void RaiseLift(double liftHeightTarget) {
        liftCurrentHeight = Robot.liftMotor.getCurrentPosition() / liftTicksPerInch;
        turretCurrentDegrees = Robot.turretMotor.getCurrentPosition() / turretTicksPerDegree;

        Robot.liftMotor.setTargetPosition((int) (liftHeightTarget * liftTicksPerInch));
        Robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Robot.liftMotor.setPower(liftSpeedUp);




    }
    public void ChangeGripperState(double gripperTarget) {
        if (grabberServoCurrentPos == grabberServoOpenPos) {
            grabberServoCurrentPos = grabberServoClosedPos;
        } else {
            grabberServoCurrentPos = grabberServoOpenPos;
        }
        Robot.grabberServo.setPosition(grabberServoCurrentPos);
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

    public void turnToHeading(double targetHeading) {
        boolean turnLeft;
        double currentHeading;
        double degreesToTurn;
        double wheelPower = 0;
        double setWheelPower = 0;
        double accelerationIncrement = 0.03;
        double desiredWheelPower;

        currentHeading = 0;//getCurrentHeading();

        degreesToTurn = Math.abs(targetHeading - currentHeading);

        turnLeft = targetHeading > currentHeading;

        if (degreesToTurn > 180) {
            turnLeft = !turnLeft;
            degreesToTurn = 360 - degreesToTurn;
        }

        while (degreesToTurn > .5 ) {//&& opModeIsActive()) {
            desiredWheelPower = (Math.pow((degreesToTurn) / 35, 4) + 5) / 100;

            if (wheelPower < desiredWheelPower) {
                wheelPower += accelerationIncrement;  // accelerate gradually
                if (wheelPower > desiredWheelPower) {
                    wheelPower = desiredWheelPower;
                }
            } else {
                wheelPower = desiredWheelPower;  // decelerate it immediately
            }

            setWheelPower = wheelPower;
            if (turnLeft) {
                setWheelPower = -setWheelPower;
            }

            /*Robot.frontLeft.setPower(-setWheelPower);
            Robot.frontRight.setPower(setWheelPower);
            Robot.backLeft.setPower(-setWheelPower);
            Robot.backRight.setPower(setWheelPower);*/

            currentHeading = 0;//getCurrentHeading();
            degreesToTurn = Math.abs(targetHeading - currentHeading);

            turnLeft = targetHeading > currentHeading;
            if (degreesToTurn > 180) {
                turnLeft = !turnLeft;
                degreesToTurn = 360 - degreesToTurn;
            }
        }

        /*Robot.frontLeft.setPower(0);
        Robot.frontRight.setPower(0);
        Robot.backLeft.setPower(0);
        Robot.backRight.setPower(0);*/
    }

    public void ZeroPowerToFloat() {
        Robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
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

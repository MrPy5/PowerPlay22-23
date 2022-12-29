package org.firstinspires.ftc.teamcode.hardware.robot;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Robot {

    public HardwareMap hardwareMap;

    //Driving
    public DcMotorEx backLeft;
    public DcMotorEx backRight;
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public double wheelCountsPerRevolution = 537.6;
    public double wheelDiameter = 3.77;
    public double ticksPerInch = (wheelCountsPerRevolution) /
            (wheelDiameter * Math.PI);

    public double deadStickZone = 0.01;
    public double wheelPowerMinToMove = 0.05;
    
    //Odometer
    public DcMotorEx odometerLeft;
    public DcMotorEx odometerRight;
    public double odometerCountsPerRevolution = 8192;
    public double odometerWheelDiameter = 1.436;
    public double odometerTicksPerInch = (odometerCountsPerRevolution) /
            (odometerWheelDiameter * Math.PI);
    //Slow Mode
    public double slowModeSpeed = .8;
    public double slowModeSlow = .4;
    public double slowModeFast = 1;
    public double slowModeGroundJuctionSlow = .2;

    public double slowModeTurnSpeed = 0.6;
    public double slowModeTurnSlow = 0.5;
    public double slowModeTurnFast = 0.6;


    //Lift
    public DcMotorEx liftMotor;
    public double liftMotorTicksPerRevolution = 537;
    public double liftSpoolDiameter = 1;
    public double liftCascadeMultiplier = 3;
    public double liftTicksPerInch = liftMotorTicksPerRevolution / (liftSpoolDiameter * Math.PI * liftCascadeMultiplier);

    public double liftPickupHeight = 0;
    public double liftJunctionGroundHeight = 2;
    public double liftUprightHeight = 2.25;
    public double liftJunctionLowHeight = 14.5;
    public double liftJunctionMediumHeight = 24.5; //24
    public double liftJunctionHighHeight = 33.5;
    public double liftMinHeightForTurning = 6;
    public double liftMaximumHeight = 36;

    public double liftSpeedUp = 1;
    public double liftSpeedDown = .7;
    public double manualLiftIncrement = 1.1;
    public int liftTimerForGuide = 70;



    //Turret
    public DcMotorEx turretMotor;
    public double turretTicksPerRevolution = 2786.0;
    public double turretTicksPerDegree = turretTicksPerRevolution / 360.0;

    public double turretForwardDegrees = 0;
    public double turretRightDegrees = 90;
    public double turretLeftDegrees = -90;
    public double turretBackDegrees = 180;

    public double turretSpeed = 0.5;
    public double turretCloseToZero = 70;

    //Grabber
    public Servo grabberServo;
    public double grabberServoClosedPos = 0.27;
    public double grabberServoHalfwayPos = 0.60;
    public double grabberServoUprightPos = 0.50;
    public double grabberServoOpenPos = 0.83;

    //Guide
    public Servo guideServo;
    public double guideServoDown = 0;
    public double guideServoUp = 0.7;
    public double guideServoDeployHeight = 10;

    //Color Sensor
    public ColorSensor colorSensorLeft;
    public ColorSensor colorSensorRight;

    //Pole Color Sensor
    public ColorSensor colorSensorPole;
    public double colorThreshold = 150;

    //Gamepad
    public double triggerSensitivity = 0.01;




    public Robot(HardwareMap robot_hardwareMap, boolean isTeleop) {
        hardwareMap = robot_hardwareMap;

        //Driving
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");


        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (!isTeleop) {
            //Odometer
            odometerLeft = hardwareMap.get(DcMotorEx.class, "odometerLeft");
            odometerRight = hardwareMap.get(DcMotorEx.class, "odometerRight");

            odometerRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            odometerLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            odometerRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            odometerLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            odometerRight.setDirection(DcMotorSimple.Direction.REVERSE);

            odometerLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        //Turret
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");

        //Lift
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        

        if (!isTeleop) {
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Grabber
        grabberServo = hardwareMap.get(Servo.class, "gripperServo");

        //Guide
        guideServo = hardwareMap.get(Servo.class, "guideServo");

        if (!isTeleop) {
            //Color Sensor
            colorSensorLeft = hardwareMap.get(ColorSensor.class, "sensorColorLeft");
            colorSensorRight = hardwareMap.get(ColorSensor.class, "sensorColorRight");
        }

        //Color Sensor Pole
        colorSensorPole = hardwareMap.get(ColorSensor.class, "sensorColorPole");
    }
}

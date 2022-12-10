package org.firstinspires.ftc.teamcode.hardware.robot;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {

    public HardwareMap hardwareMap;

    //Driving
    public static DcMotorEx backLeft;
    public static DcMotorEx backRight;
    public static DcMotorEx frontLeft;
    public static DcMotorEx frontRight;
    public static double wheelCountsPerRevolution = 537.6;
    public static double wheelDiameter = 3.77;
    public static double ticksPerInch = (wheelCountsPerRevolution) /
            (wheelDiameter * Math.PI);

    public static double deadStickZone = 0.01;
    public static double wheelPowerMinToMove = 0.05;


    //Slow Mode
    public static double slowModeSpeed = .8;
    public static double slowModeSlow = .4;
    public static double slowModeFast = .8;
    public static double slowModeGroundJuctionSlow = .2;

    public static double slowModeTurnSpeed = 0.6;
    public static double slowModeTurnSlow = 0.5;
    public static double slowModeTurnFast = 0.6;


    //Lift
    public static DcMotorEx liftMotor;
    public static double liftMotorTicksPerRevolution = 537;
    public static double liftSpoolDiameter = 1;
    public static double liftCascadeMultiplier = 3;
    public static double liftTicksPerInch = liftMotorTicksPerRevolution / (liftSpoolDiameter * Math.PI * liftCascadeMultiplier);

    public static double liftPickupHeight = 0;
    public static double liftJunctionGroundHeight = 2;
    public static double liftJunctionLowHeight = 14.5;
    public static double liftJunctionMediumHeight = 24;
    public static double liftJunctionHighHeight = 33.5;
    public static double liftMinHeightForTurning = 6;
    public static double liftMaximumHeight = 34;

    public static double liftSpeedUp = 1;
    public static double liftSpeedDown = .7;
    public static double manualLiftIncrement = 2.5;
    public static int liftTimerForGuide = 70;



    //Turret
    public static DcMotorEx turretMotor;
    public static double turretTicksPerRevolution = 2786.0;
    public static double turretTicksPerDegree = turretTicksPerRevolution / 360.0;

    public static double turretForwardDegrees = 0;
    public static double turretRightDegrees = 90;
    public static double turretLeftDegrees = -90;
    public static double turretBackDegrees = 180;

    public static double turretSpeed = 0.5;
    public static double turretCloseToZero = 70;

    //Grabber
    public static Servo grabberServo;
    public static double grabberServoClosedPos = 0.18;
    public static double grabberServoHalfwayPos = 0.55;
    public static double grabberServoOpenPos = 0.7;

    //Guide
    public static Servo guideServo;
    public static double guideServoDown = 0.02;
    public static double guideServoUp = 0.86;
    public static double guideServoDeployHeight = 10;

    //Color Sensor
    public static ColorSensor colorSensorLeft;
    public static ColorSensor colorSensorRight;

    //Pole Color Sensor
    public static ColorSensor colorSensorPole;
    public static double colorThreshold = 500;

    //Gamepad
    public static double triggerSensitivity = 0.01;


    public Robot(HardwareMap robot_hardwareMap) {
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

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //Turret
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Lift
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Grabber
        grabberServo = hardwareMap.get(Servo.class, "gripperServo");

        //Guide
        guideServo = hardwareMap.get(Servo.class, "guideServo");

        //Color Sensor
        colorSensorLeft = hardwareMap.get(ColorSensor.class, "sensorColorLeft");
        colorSensorRight = hardwareMap.get(ColorSensor.class, "sensorColorRight");

        //Color Sensor Pole
        colorSensorPole = hardwareMap.get(ColorSensor.class, "sensorColorPole");
    }
}

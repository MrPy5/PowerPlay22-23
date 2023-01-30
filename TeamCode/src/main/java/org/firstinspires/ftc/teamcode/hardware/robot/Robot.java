package org.firstinspires.ftc.teamcode.hardware.robot;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {


    //---DRIVING---//
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

        //Odometer
        public static DcMotorEx odometerLeft;
        public static DcMotorEx odometerRight;
        public static double odometerCountsPerRevolution = 8192;
        public static double odometerWheelDiameter = 1.436;
        public static double odometerTicksPerInch = (odometerCountsPerRevolution) /
                (odometerWheelDiameter * Math.PI);
        public static double workingEncoderVelocityDifference = 4;


    //---MOTORS---//
        //Lift
        public static DcMotorEx liftMotor;
        public static double liftMotorTicksPerRevolution = 537;
        public static double liftSpoolDiameter = 1;
        public static double liftCascadeMultiplier = 3;
        public static double liftTicksPerInch = liftMotorTicksPerRevolution / (liftSpoolDiameter * Math.PI * liftCascadeMultiplier);

        public static double liftPickupHeight = 0;
        public static double liftJunctionGroundHeight = 2;
        public static double liftConeUprightHeight = 4;
        public static double liftJunctionLowHeight = 14.5;
        public static double liftJunctionMediumHeight = 24.5; //24
        public static double liftJunctionHighHeight = 33;
        public static double liftMinHeightForTurning = 6;
        public static double liftMaximumHeight = 36;

        public static double liftSpeedUp = 1.0;
        public static double liftSpeedDown = 0.7;
        public static double manualLiftIncrement = 1.4;
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


    //---SERVOS---//
        //Grabber
        public static Servo grabberServo;
        public static double grabberServoClosedPos = 0.25;
        public static double grabberServoHalfwayPos = 0.60;
        public static double grabberServoUprightPos = 0.47;
        public static double grabberServoOpenPos = 0.70;

        //Guide
        public static Servo guideServo;
        public static double guideServoDown = 0.2;
        public static double guideServoUp = guideServoDown + 0.65;
        public static double guideServoDeployHeight = 10;

        //Cone-Upright
        public static Servo coneUprightLeftServo;
        public static double cULeftClosedPos = .15;
        public static double cULeftOpenPos = cULeftClosedPos + .54;
        public static double cULeftFlickPos = cULeftClosedPos + .73;

        public static Servo coneUprightRightServo;
        public static double cURightClosedPos = .89;
        public static double cURightOpenPos = cURightClosedPos - .53;
        public static double cURightFlickPos = cURightClosedPos - .73;


    //---SENSORS---//
        //Color Sensors
        public static ColorSensor colorSensorLeft;
        public static ColorSensor colorSensorRight;

        //Pole Color Sensor
        public static ColorSensor colorSensorPole;
        public static double colorThreshold = 300;


    //---MISC---//
        //Gamepad
        public static double triggerSensitivity = 0.01;
        //Hardware Map
        public HardwareMap hardwareMap;



    public Robot(HardwareMap robot_hardwareMap, boolean isTeleop) {
        //Set Hardware map
        hardwareMap = robot_hardwareMap;

        //---Driving---//
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

        //---Odometer---//
        odometerLeft = hardwareMap.get(DcMotorEx.class, "odometerLeft");
        odometerRight = hardwareMap.get(DcMotorEx.class, "odometerRight");

        odometerRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odometerLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odometerRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometerLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        odometerRight.setDirection(DcMotorSimple.Direction.REVERSE);

        odometerLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //---Turret---//
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");

        if (!isTeleop) {
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //---Lift---/
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");

        if (!isTeleop) {
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //---Grabber---//
        grabberServo = hardwareMap.get(Servo.class, "gripperServo");

        //---Guide---//
        guideServo = hardwareMap.get(Servo.class, "guideServo");

        //---UprightServo---//
        coneUprightLeftServo = hardwareMap.get(Servo.class, "coneUprightLeftServo");
        coneUprightRightServo = hardwareMap.get(Servo.class, "coneUprightRightServo");

        //---Color Sensor---//
        colorSensorLeft = hardwareMap.get(ColorSensor.class, "sensorColorLeft");
        colorSensorRight = hardwareMap.get(ColorSensor.class, "sensorColorRight");

        //---Color Sensor Pole---//
        colorSensorPole = hardwareMap.get(ColorSensor.class, "sensorColorPole");
    }
}

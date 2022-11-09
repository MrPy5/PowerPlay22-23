package org.firstinspires.ftc.teamcode.hardware.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {

    public HardwareMap hardwareMap;


    public static DcMotorEx backLeft;
    public static DcMotorEx backRight;
    public static DcMotorEx frontLeft;
    public static DcMotorEx frontRight;
    public static double wheelCountsPerRevolution = 537.6;
    public static double wheelDiameter = 3.77;
    public static double ticksPerInch = (wheelCountsPerRevolution) /
            (wheelDiameter * Math.PI);


    public static double wheelBase = 12.5;
    public static double radiusOfWheelBaseCircle = Math.hypot(wheelBase, wheelBase);
    public static double wheelBaseCircumference = radiusOfWheelBaseCircle * Math.PI;
    public static double inchesPerWheelBaseDegree = wheelBaseCircumference / 360;



    public static DcMotorEx liftMotor;
    public static double liftMotorTicksPerRevolution = 537;
    public static double liftSpoolDiameter = 0.94; //inches - if you have the correct spool diameter, everything else should just work
    public static double liftCascadeMultiplier = 3; // 3 stages of cascade stringing
    public static double liftTicksPerInch = liftMotorTicksPerRevolution / (liftSpoolDiameter * Math.PI * liftCascadeMultiplier);

    public static double liftPickupHeight = 0;
    public static double liftJunctionGroundHeight = 2;
    public static double liftJunctionLowHeight = 15;
    public static double liftJunctionMediumHeight = 24;
    public static double liftJunctionHighHeight = 34;
    public static double liftMinHeightForTurning = 6;
    public static double liftMaximumHeight = 36;

    public static DcMotorEx turretMotor;
    public static double turretTicksPerRevolution = 2786.0;
    public static double turretTicksPerDegree = turretTicksPerRevolution / 360.0;

    public static double turretForwardDegrees = 0; //all rotation variables in degrees
    public static double turretRightDegrees = 90;
    public static double turretLeftDegrees = -90;
    public static double turretBackDegrees = 180;

    public static Servo grabberServo;
    public static double grabberServoClosedPos = 0.18;
    public static double grabberServoOpenPos = 0.7;


    public Robot(HardwareMap robot_hardwareMap) {
        hardwareMap = robot_hardwareMap;

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


        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        grabberServo = hardwareMap.get(Servo.class, "gripperServo");


    }
}

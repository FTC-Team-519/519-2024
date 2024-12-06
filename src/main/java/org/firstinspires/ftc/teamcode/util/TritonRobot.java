package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.*;

import java.util.ArrayList;
import java.util.Arrays;

public class TritonRobot {
    ColorRangeSensor colorSensor;
    TouchSensor touchFront;
    TouchSensor touchBack;

    DcMotor frontLeft = null;
    DcMotor frontRight = null;
    DcMotor backLeft = null;
    DcMotor backRight = null;
    ArrayList<DcMotor> wheels;

    DcMotor rightArmMotor = null;
    DcMotor leftArmMotor = null;

    DcMotor rightSpool = null;
    DcMotor leftSpool = null;

    CRServo rightIntakeWheel = null;
    CRServo leftIntakeWheel = null;

    ArrayList<DcMotor> motors;

    // init robot
    TritonRobot(HardwareMap hardwareMap) {
        // Wheels
        frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        backRight = hardwareMap.get(DcMotor.class,"backRight");
        backLeft = hardwareMap.get(DcMotor.class,"backLeft");

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        wheels = new ArrayList<DcMotor>();
        wheels.addAll(Arrays.asList(frontRight,frontLeft,backRight,backLeft));

        // Arm
        rightArmMotor = hardwareMap.get(DcMotor.class,"rightArmMotor");
        leftArmMotor = hardwareMap.get(DcMotor.class,"leftArmMotor");

        rightArmMotor.setDirection(DcMotor.Direction.FORWARD);
        leftArmMotor.setDirection(DcMotor.Direction.REVERSE);

        // Arm extension
        rightSpool = hardwareMap.get(DcMotor.class,"rightSpool");
        leftSpool = hardwareMap.get(DcMotor.class,"leftSpool");

        rightSpool.setDirection(DcMotor.Direction.FORWARD);
        leftSpool.setDirection(DcMotor.Direction.REVERSE);

        // Intake
        rightIntakeWheel = hardwareMap.get(CRServo.class,"rightIntakeWheel");
        leftIntakeWheel = hardwareMap.get(CRServo.class,"leftIntakeWheel");

        rightIntakeWheel.setDirection(CRServo.Direction.FORWARD);
        leftIntakeWheel.setDirection(CRServo.Direction.FORWARD);

        // Sensors
        colorSensor = hardwareMap.get(ColorRangeSensor.class,"colorSensor");

        touchFront = hardwareMap.get(TouchSensor.class,"touchFront");
        touchBack = hardwareMap.get(TouchSensor.class,"touchBack");


        motors = new ArrayList<DcMotor>();
        motors.addAll(Arrays.asList(frontLeft,frontRight,backLeft,backRight,rightArmMotor,leftArmMotor,rightSpool,leftSpool));

        for (DcMotor motor : motors) { // small thingy to simplify code to make all motors brake on zero power
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public boolean setTargetPositionsForMotors(double position) {
        return(frontLeft.getTargetPosition()==position && backLeft.getTargetPosition()==position &&
                frontRight.getTargetPosition()==position && backRight.getTargetPosition()==position);
    }

    public static double getCountsPerInchForMotors() {
        final double     COUNTS_PER_MOTOR_REV    = 384.5  ;    // eg: Using 5203 Yellowjacket 435 RPM w/ given encoder
        final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // Simple Bevel Gear ratio is 2:1
        final double     WHEEL_DIAMETER_INCHES   = 4;     // For figuring circumference
        return (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    }
}

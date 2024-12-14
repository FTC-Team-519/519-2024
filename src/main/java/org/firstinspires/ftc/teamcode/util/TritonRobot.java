package org.firstinspires.ftc.teamcode.util;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

    //positive value moves arm up
    //negative value moves arm down
    DcMotor rightArmMotor = null;
    DcMotor leftArmMotor = null;

    //positive extends arm
    //negative retracts arm
    DcMotor rightSpool = null;
    DcMotor leftSpool = null;

    CRServo rightIntakeWheel = null;
    CRServo leftIntakeWheel = null;

    ArrayList<DcMotor> motors;

    AnalogInput potentiometer = null;

    public enum SampleColor {
        RED,
        BLUE,
        YELLOW
    }

    // init robot
    public TritonRobot(HardwareMap hardwareMap) {
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

        rightArmMotor.setDirection(DcMotor.Direction.REVERSE);
        leftArmMotor.setDirection(DcMotor.Direction.FORWARD);

        // Arm extension
        rightSpool = hardwareMap.get(DcMotor.class,"rightSpool");
        leftSpool = hardwareMap.get(DcMotor.class,"leftSpool");

        rightSpool.setDirection(DcMotor.Direction.REVERSE);
        leftSpool.setDirection(DcMotor.Direction.FORWARD);

        // Intake
        rightIntakeWheel = hardwareMap.get(CRServo.class,"rightIntakeWheel");
        leftIntakeWheel = hardwareMap.get(CRServo.class,"leftIntakeWheel");

        rightIntakeWheel.setDirection(CRServo.Direction.FORWARD);
        leftIntakeWheel.setDirection(CRServo.Direction.REVERSE);

        // Sensors
        colorSensor = hardwareMap.get(ColorRangeSensor.class,"colorSensor");

        touchFront = hardwareMap.get(TouchSensor.class,"touchFront");
        touchBack = hardwareMap.get(TouchSensor.class,"touchBack");

        potentiometer = hardwareMap.get(AnalogInput.class,"potentiometer");


        motors = new ArrayList<DcMotor>();
        motors.addAll(Arrays.asList(frontLeft,frontRight,backLeft,backRight,rightArmMotor,leftArmMotor,rightSpool,leftSpool));

        for (DcMotor motor : motors) { // small thingy to simplify code to make all motors brake on zero power
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    //arm rotation motor stuff
    public void setArmRotateMode(DcMotor.RunMode runMode){
        this.leftArmMotor.setMode(runMode);
        this.rightArmMotor.setMode(runMode);
    }

    public void setArmRotatePower(double power){
        this.leftArmMotor.setPower(power);
        this.rightArmMotor.setPower(power);
    }

    //drive motor stuff
    public boolean isTargetPositionsForDriveMotorsSet(int position) {
        return(frontLeft.getTargetPosition()==position && backLeft.getTargetPosition()==position &&
                frontRight.getTargetPosition()==position && backRight.getTargetPosition()==position);
    }

    public static double getCountsPerInchForDriveMotors() {
        final double     COUNTS_PER_MOTOR_REV    = 384.5  ;    // eg: Using 5203 Yellowjacket 435 RPM w/ given encoder
        final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // Simple Bevel Gear ratio is 2:1
        final double     WHEEL_DIAMETER_INCHES   = 4;     // For figuring circumference
        return (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    }

    public void setDriveMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    public void setLeftDrivePower(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
    }

    public void setRightDrivePower(double power) {
        frontRight.setPower(power);
        backRight.setPower(power);
    }

    public void setAllDrivePower(double power) {
        setLeftDrivePower(power);
        setRightDrivePower(power);
    }

    public void setDriveTargetPosition(int position) {
        frontLeft.setTargetPosition(position);
        frontRight.setTargetPosition(position);
        backLeft.setTargetPosition(position);
        backRight.setTargetPosition(position);
    }

    public boolean atDriveTargetPosition(boolean isGoingForward) {
        if (isGoingForward) {
            return atDriveTargetPositionForward();
        }
        else {
            return atDriveTargetPositionBackward();
        }
    }

    public boolean atDriveTargetPositionForward() {
        return(frontLeft.getCurrentPosition() >= frontLeft.getTargetPosition() &&
                frontRight.getCurrentPosition() >= frontRight.getTargetPosition() &&
                backLeft.getCurrentPosition() >= backLeft.getTargetPosition() &&
                backRight.getCurrentPosition() >= backRight.getTargetPosition());
    }

    public boolean atDriveTargetPositionBackward() {
        return(frontLeft.getCurrentPosition() <= frontLeft.getTargetPosition() &&
                frontRight.getCurrentPosition() <= frontRight.getTargetPosition() &&
                backLeft.getCurrentPosition() <= backLeft.getTargetPosition() &&
                backRight.getCurrentPosition() <= backRight.getTargetPosition());
    }

    // Drive for non-drive motors

    public void setSpoolPower(double power) {
        leftSpool.setPower(power);
        rightSpool.setPower(power);
    }

    public void setIntakePower(double power) {
        leftIntakeWheel.setPower(power);
        rightIntakeWheel.setPower(power);
    }

    // Returns if a piece is currently in the intake or not based on colorSensor and given range
    // This has the visibleDistance as a variable in case we want to further refine the range later on
    public boolean doesIntakeContainPiece(double visibleDistance) {
        return(colorSensor.getDistance(DistanceUnit.INCH)<visibleDistance);
    }

    // Returns a SampleColor to allow for easy case/switch statements (RED, BLUE, and YELLOW are the cases)
    public SampleColor currentIntakePieceColor(){
        colorSensor.enableLed(true);

        float[] hsvValues = {0F,0F,0F};
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();
        Color.RGBToHSV(red * 8, green * 8, blue * 8, hsvValues);

        float hue = hsvValues[0];

        // This is a range due to amount of light, also red range was lowered due to overriding yellow outputs.
        // Yellow min is 10 from experimentation, Red always reads as 0, so 5 to allow for some room.
        // Blue had no changes due to being far enough away from Red and Yellow that it had no effect.
        // NOTE: This is due to the light being off. Further changes are necessary. Will delete this when
        //  done with said changes. (If somebody else fixes this please delete this note)

        if(hue<34) {
            colorSensor.enableLed(false);
            return SampleColor.RED;
        }
        else if(hue>100) {
            colorSensor.enableLed(false);
            return SampleColor.BLUE;
        }
        else {
            colorSensor.enableLed(false);
            return SampleColor.YELLOW;
        }
    }

    // Returns based on whether the touch sensors are pressed
    // This is majorly used as a limiting function for the arm rotation
    public boolean touchingBack() {
        return touchBack.isPressed();
    }

    public boolean touchingFront() {
        return touchFront.isPressed();
    }

    // This is unfinished, but should be used as a way to get the current position of the potentiometer
    // or absolute encoder. Essentially it will get the current position of the arm, regardless of when
    // the robot was last started, as it doesn't reset like the motor encoders do.
    public double readAbsoluteEncoderValue() {
        return potentiometer.getVoltage();
    }

    //bunch of getter methods
    //generally, try to avoid the motor getters
    public ColorRangeSensor getColorSensor() {
        return colorSensor;
    }

    public TouchSensor getTouchFront() {
        return touchFront;
    }

    public TouchSensor getTouchBack() {
        return touchBack;
    }

    public DcMotor getFrontLeft() {
        return frontLeft;
    }

    public DcMotor getFrontRight() {
        return frontRight;
    }

    public DcMotor getBackLeft() {
        return backLeft;
    }

    public DcMotor getBackRight() {
        return backRight;
    }

    public ArrayList<DcMotor> getWheels() {
        return wheels;
    }

    public DcMotor getRightArmMotor() {
        return rightArmMotor;
    }

    public DcMotor getLeftArmMotor() {
        return leftArmMotor;
    }

    public DcMotor getRightSpool() {
        return rightSpool;
    }

    public DcMotor getLeftSpool() {
        return leftSpool;
    }

    public CRServo getRightIntakeWheel() {
        return rightIntakeWheel;
    }

    public CRServo getLeftIntakeWheel() {
        return leftIntakeWheel;
    }

    public ArrayList<DcMotor> getMotors() {
        return motors;
    }
}

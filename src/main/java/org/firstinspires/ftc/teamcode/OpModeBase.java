package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;
import java.util.List;

public abstract class OpModeBase extends OpMode {

    protected boolean isTimeToRun = true;

    protected DcMotor frontRight = null;
    protected DcMotor frontLeft = null;
    protected DcMotor backRight = null;
    protected DcMotor backLeft = null;
    protected List<DcMotor> motors;

    public static final double     COUNTS_PER_MOTOR_REV    = 5281.1  ;    // eg: Using 5202 Yellowjacket 30 RPM w/ given encoder
    public static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // Simple Bevel Gear ratio is 2:1
    public static final double     WHEEL_DIAMETER_INCHES   = 4;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    protected double  driveSpeed    = 0.25;

    @Override
    public void init() {

        frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        backRight = hardwareMap.get(DcMotor.class,"backRight");
        backLeft = hardwareMap.get(DcMotor.class,"backLeft");
        motors = Arrays.asList(frontLeft,frontRight,backLeft,backRight);

        for (DcMotor motor : motors) { // small thingy to simplify code to make all motors brake on zero power
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public abstract void loop();

    protected boolean driveDistanceInches(double speed, double distanceInches) {
//*       Returns if done (Drives forward)
//        Should only run the first time
        if (!allTargetPositionsSet((int) (distanceInches * COUNTS_PER_INCH))) {
            setTargetPosition((int) (distanceInches * COUNTS_PER_INCH));
            allDrivePower(speed);
        }

        if (atTargetPosition(distanceInches >= 0)){
            return true;
        }
        return false;
    }

    protected boolean allTargetPositionsSet(int position) {
        return(frontLeft.getTargetPosition()==position && backLeft.getTargetPosition()==position &&
                frontRight.getTargetPosition()==position && backRight.getTargetPosition()==position);
    }

    protected boolean atTargetPosition(boolean isGoingForward) {
        if (isGoingForward) {
            return atTargetPositionForward();
        }
        else {
            return atTargetPositionBackward();
        }
    }

    protected boolean atTargetPositionForward() {
        return(frontLeft.getCurrentPosition() >= frontLeft.getTargetPosition() &&
                frontRight.getCurrentPosition() >= frontRight.getTargetPosition() &&
                backLeft.getCurrentPosition() >= backLeft.getTargetPosition() &&
                backRight.getCurrentPosition() >= backRight.getTargetPosition());
    }

    protected boolean atTargetPositionBackward() {
        return(frontLeft.getCurrentPosition() <= frontLeft.getTargetPosition() &&
                frontRight.getCurrentPosition() <= frontRight.getTargetPosition() &&
                backLeft.getCurrentPosition() <= backLeft.getTargetPosition() &&
                backRight.getCurrentPosition() <= backRight.getTargetPosition());
    }

    protected void setTargetPosition(int position) {
        frontLeft.setTargetPosition(position);
        frontRight.setTargetPosition(position);
        backLeft.setTargetPosition(position);
        backRight.setTargetPosition(position);
    }

    protected void leftDrivePower(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
    }

    protected void rightDrivePower(double power) {
        frontRight.setPower(power);
        backRight.setPower(power);
    }

    protected void allDrivePower(double power) {
        leftDrivePower(power);
        rightDrivePower(power);
    }

    protected void setMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }
}

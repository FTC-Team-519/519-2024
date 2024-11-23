package org.firstinspires.ftc.teamcode.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;
import java.util.List;

@Autonomous(name="Drive2FeetForward",group="Autonomous")
public class DriveForward2Feet extends OpMode {

    private boolean isTimeToRun = true;

    private DcMotor frontRight = null;
    private DcMotor frontLeft = null;
    private DcMotor backRight = null;
    private DcMotor backLeft = null;
    private List<DcMotor> motors;

    private static final double     COUNTS_PER_MOTOR_REV    = 5281.1  ;    // eg: Using 5202 Yellowjacket 30 RPM w/ given encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // Simple Bevel Gear ratio is 2:1
    private static final double     WHEEL_DIAMETER_INCHES   = 4;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    protected double  driveSpeed    = 0.25;

    @Override
    public void init(){

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
    public void loop() {
        if(isTimeToRun) {
            boolean isDone = driveDistanceInches(driveSpeed, 24);
            if(isDone) {
                isTimeToRun = false;
            }
        }

    }

    protected boolean driveDistanceInches(double speed, double distanceInches) {
//*       Returns if done (Drives forward)
//        Should only run the first time
        if (!allTargetPositionsSet((int)(distanceInches * COUNTS_PER_INCH))) {
            setTargetPosition((int) (distanceInches * COUNTS_PER_INCH));
            allDrivePower(speed);
        }

        if (atTargetPositionForward()) {
            return true;
        }
        return false;
    }

    protected boolean allTargetPositionsSet(int position) {
        return(frontLeft.getTargetPosition()==position && backLeft.getTargetPosition()==position &&
                frontRight.getTargetPosition()==position && backRight.getTargetPosition()==position);
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

package org.firstinspires.ftc.teamcode.autonomous;



//DO NOT RUN THIS CODE UNTIL FURTHER NOTICE
//This code is built to run on the back limit switch, which is in a position that would tear the robot apart
// if it runs through this code.
//Currently it should work, although I was not willing to allow it to go through
//AKA
//DO NOT RUN THIS CODE UNLESS THE BACK LIMIT SWITCH IS IN A POSITION THAT WONT TEAR IT APART
//When I know that this code is safe to use with the robot, I will commit a version that has these comments removed













import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
@Disabled
@Autonomous(name="ArmMotorAndExtensionAuto",group="Autonomous")
public class ArmMotorAndExtensionAuto extends OpMode {

    private boolean hasReset;

    private static final double COUNTS_PER_MOTOR_REV    = 5281.1  ;    // eg: Using 5202 Yellowjacket 30 RPM w/ given encoder
    private static final double GEAR_RATIO = 3;
    private static final double AMOUNT_PER_ROTATION = COUNTS_PER_MOTOR_REV * GEAR_RATIO;
    private static final double drivePower = 0.25;

    private TouchSensor touchFront = null;
    private TouchSensor touchBack = null;

    private DcMotor rightArmMotor = null;
    private DcMotor leftArmMotor = null;

    private enum armSteps  {
        CHECK_UNTIL_PRESSED,
        GO_TO_POSITION,
        DONE
    }

    private armSteps currStep = armSteps.CHECK_UNTIL_PRESSED;

    public void init(){
        hasReset = false;

        currStep = armSteps.CHECK_UNTIL_PRESSED;

        touchFront = hardwareMap.get(TouchSensor.class,"touchFront");
        touchBack = hardwareMap.get(TouchSensor.class,"touchBack");

        rightArmMotor = hardwareMap.get(DcMotor.class,"rightArmMotor");
        leftArmMotor = hardwareMap.get(DcMotor.class,"leftArmMotor");

        rightArmMotor.setDirection(DcMotor.Direction.FORWARD);
        leftArmMotor.setDirection(DcMotor.Direction.REVERSE);

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE);

        setTargetPosition(0);


    }

    @Override
    public void loop() {

        switch (currStep) {
            case CHECK_UNTIL_PRESSED: {
                setPower(-drivePower);

                if(checkBack()) {
                    currStep = armSteps.GO_TO_POSITION;
                }
                break;
            }
            case GO_TO_POSITION: {
                if(!hasReset){
                    setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    hasReset = true;
                }
                else {
                    boolean hasFinished = goToPosition(1/18);
                    if(hasFinished) {
                        currStep = armSteps.DONE;
                    }
                }
                break;
            }
        }
        telemetry.addData("Current Left Position:",leftArmMotor.getCurrentPosition());
        telemetry.addData("Current Right Position:",rightArmMotor.getCurrentPosition());
        telemetry.addData("Target Left Position:",leftArmMotor.getTargetPosition());
        telemetry.addData("Target Right Position:",rightArmMotor.getTargetPosition());
        telemetry.addData("Current Step:",currStep);

    }

    protected boolean goToPosition(double position) {
        if(!hasSetTargetPosition((int) (position * AMOUNT_PER_ROTATION))) {
            setTargetPosition((int) (position * AMOUNT_PER_ROTATION));
            setPower(drivePower);
        }

        return atTargetPosition();
    }

    protected boolean atTargetPosition() {
        return (rightArmMotor.getCurrentPosition() >= rightArmMotor.getTargetPosition() &&
                leftArmMotor.getCurrentPosition() >= leftArmMotor.getTargetPosition());
    }

    protected boolean hasSetTargetPosition(int position) {
        return (rightArmMotor.getTargetPosition()==position && leftArmMotor.getTargetPosition()==position);
    }

    protected void setTargetPosition(int position) {
        rightArmMotor.setTargetPosition(position);
        leftArmMotor.setTargetPosition(position);
    }

    protected void setMode(DcMotor.RunMode mode) {
        rightArmMotor.setMode(mode);
        leftArmMotor.setMode(mode);
    }

    protected void setPower(double power) {
        rightArmMotor.setPower(power);
        leftArmMotor.setPower(power);
    }

    protected void setZeroPowerMode(DcMotor.ZeroPowerBehavior mode) {
        rightArmMotor.setZeroPowerBehavior(mode);
        leftArmMotor.setZeroPowerBehavior(mode);
    }

    protected boolean checkBack() {
        return touchBack.isPressed();
    }

    protected boolean checkFront() {
        return touchFront.isPressed();
    }

}

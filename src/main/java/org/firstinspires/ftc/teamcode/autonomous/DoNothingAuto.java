package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;
import java.util.List;


@Autonomous(name= "Do Nothing",group="Autonomous")
//@Disabled
public class DoNothingAuto extends OpMode {
    private DcMotor frontRight = null;
    private DcMotor frontLeft = null;
    private DcMotor backRight = null;
    private DcMotor backLeft = null;
    private DcMotor rightArmMotor = null;
    private DcMotor leftArmMotor = null;
    private DcMotor rightSpool = null;
    private DcMotor leftSpool = null;
    private CRServo rightIntakeWheel = null;
    private CRServo leftIntakeWheel = null;
    private List<DcMotor> motors;
    @Override
    public void init() {
        frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        backRight = hardwareMap.get(DcMotor.class,"backRight");
        backLeft = hardwareMap.get(DcMotor.class,"backLeft");
        rightArmMotor = hardwareMap.get(DcMotor.class,"rightArmMotor");
        leftArmMotor = hardwareMap.get(DcMotor.class,"leftArmMotor");
        rightSpool = hardwareMap.get(DcMotor.class,"rightSpool");
        leftSpool = hardwareMap.get(DcMotor.class,"leftSpool");
        rightIntakeWheel = hardwareMap.get(CRServo.class,"rightIntakeWheel");
        leftIntakeWheel = hardwareMap.get(CRServo.class,"leftIntakeWheel");
        motors = Arrays.asList(frontLeft,frontRight,backLeft,backRight,rightArmMotor,leftArmMotor,rightSpool,leftSpool);

        for (DcMotor motor : motors) { // small thingy to simplify code to make all motors brake on zero power
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        leftArmMotor.setDirection(DcMotor.Direction.REVERSE);
        leftSpool.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {

    }
}

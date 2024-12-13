package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.*;

@TeleOp(name="TestingDeepRobot", group="Testing")
public class TestingDeepRobot extends LinearOpMode {
    ColorRangeSensor colorSensor;
    TouchSensor touchFront;
    TouchSensor touchBack;
    VisionPortal visionPortal;
    AprilTagProcessor aprilTagProcessor;

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor rightArmMotor = null;
    private DcMotor leftArmMotor = null;
    private DcMotor rightSpool = null;
    private DcMotor leftSpool = null;
    private CRServo rightIntakeWheel = null;
    private CRServo leftIntakeWheel = null;
    private List<DcMotor> motors;

    @Override
    public void runOpMode() {

        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class,"Webcam 1"),aprilTagProcessor);

        float[] hsvValues = {0F,0F,0F};
        colorSensor = hardwareMap.get(ColorRangeSensor.class,"colorSensor");

        touchFront = hardwareMap.get(TouchSensor.class,"touchFront");
        touchBack = hardwareMap.get(TouchSensor.class,"touchBack");

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
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        rightArmMotor.setDirection(DcMotor.Direction.FORWARD);
        leftArmMotor.setDirection(DcMotor.Direction.REVERSE);
        rightSpool.setDirection(DcMotor.Direction.FORWARD);
        leftSpool.setDirection(DcMotor.Direction.REVERSE);
        rightIntakeWheel.setDirection(CRServo.Direction.REVERSE);
        leftIntakeWheel.setDirection(CRServo.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            boolean isPieceThere = false;
            final double reachableRange = 3.0;
            if(colorSensor.getDistance(DistanceUnit.INCH)<reachableRange) {
                isPieceThere = true;
            }

            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            Color.RGBToHSV(red * 8, green * 8, blue * 8, hsvValues);

            double max;

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double rightArmMove = 0.0;
            double leftArmMove = 0.0;
            double extend = 0.0;

            double outtake = gamepad1.right_trigger;
            double intake = -gamepad1.left_trigger;

            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            max = Math.max(Math.abs(leftFrontPower),Math.abs(rightFrontPower));
            max = Math.max(max,Math.abs(leftBackPower));
            max = Math.max(max,Math.abs(rightBackPower));

            if(max>1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }
            if(gamepad1.right_bumper && !touchBack.isPressed()) {
                rightArmMove = -0.7;
                leftArmMove = -0.7;
            }
            else if(gamepad1.left_bumper && !touchFront.isPressed()) {
                rightArmMove = 0.7;
                leftArmMove = 0.7;
            }
            if(gamepad1.a) {
                extend = 1.0;
            }
            else if(gamepad1.b) {
                extend = -1.0;
            }

            frontRight.setPower(rightFrontPower);
            frontLeft.setPower(leftFrontPower);
            backRight.setPower(rightBackPower);
            backLeft.setPower(leftBackPower);

            rightArmMotor.setPower(rightArmMove);
            leftArmMotor.setPower(leftArmMove);

            rightSpool.setPower(extend);
            leftSpool.setPower(extend);

            if(outtake>intake*-1) {
                rightIntakeWheel.setPower(outtake);
                leftIntakeWheel.setPower(outtake);
            }
            else {
                rightIntakeWheel.setPower(intake);
                leftIntakeWheel.setPower(intake);
            }

            if(touchFront.isPressed()){
                telemetry.addLine("Front Sensor: Is Pressed");
            }
            else{
                telemetry.addLine("Front Sensor: Is Not Pressed");
            }

            if(touchBack.isPressed()){
                telemetry.addLine("Back Sensor: Is Pressed");
            }
            else{
                telemetry.addLine("Back Sensor: Is Not Pressed");
            }

            if(isPieceThere) {
                if (hsvValues[0] < 50) {
                    telemetry.addData("Current Color", "Red");
                } else if (hsvValues[0] > 100) {
                    telemetry.addData("Current Color", "Blue");
                } else {
                    telemetry.addData("Current Color", "Yellow");
                }
            }

            telemetry.addData("Piece Is There:",isPieceThere);

            telemetry.update();

        }
        visionPortal.close();

    }

}

package org.firstinspires.ftc.teamcode.util;


import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="GeneralTeleOp",group="CompetitionTeleOp")
public class GeneralTeleOp extends OpModeBase{

    @Override
    public void loop() {


        // This section returns the current piece's color (if it exists)

        float[] hsvValues = {0F, 0F, 0F};
        int red = robot.colorSensor.red();
        int green = robot.colorSensor.green();
        int blue = robot.colorSensor.blue();
        Color.RGBToHSV(red * 8, green * 8, blue * 8, hsvValues);

        telemetry.addData("Hue: ", hsvValues[0]);

        boolean pieceIsThere = robot.doesIntakeContainPiece(3.0);

        if (pieceIsThere) {
            switch (robot.currentIntakePieceColor()) {
                case RED: {
                    telemetry.addLine("Piece Color: Red");
                    break;
                }
                case BLUE: {
                    telemetry.addLine("Piece Color: Blue");
                    break;
                }
                case YELLOW: {
                    telemetry.addLine("Piece Color: Yellow");
                    break;
                }
            }
        }
        telemetry.addData("Is Piece There: ", pieceIsThere);

        // This section sets drive motor powers, and limits them to a maximum of 1
        // This is done so they don't try to go higher than their possible speed
        double drivePowerDivisor = 1.0;

        double max;

        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        max = Math.max(leftFrontPower, rightFrontPower);
        max = Math.max(leftBackPower, max);
        max = Math.max(rightBackPower, max);

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontPower /= drivePowerDivisor;
        rightFrontPower /= drivePowerDivisor;
        leftBackPower /= drivePowerDivisor;
        rightBackPower /= drivePowerDivisor;

        robot.frontLeft.setPower(leftFrontPower);
        robot.frontRight.setPower(rightFrontPower);
        robot.backLeft.setPower(leftBackPower);
        robot.backRight.setPower(rightBackPower);

        // This section sets the arm rotation motors based on the bumpers (right is down, left is up)
        // It also incorporates the touch sensors in an effort to stop the driver from destroying the robot
        double armDriveSpeed = 0.5;

        double armMotorPower;
        double rightGiven = 0.0;
        double leftGiven = 0.0;

        if (!robot.touchingBack()) {
            if (gamepad1.left_bumper) {
                leftGiven = armDriveSpeed;
            }
        } else {
            telemetry.addLine("WARNING: Robot is at highest possible back position");
        }
        if (!robot.touchingFront()) {
            if (gamepad1.right_bumper) {
                rightGiven = armDriveSpeed;
            }
        } else {
            telemetry.addLine("WARNING: Robot is at lowest possible front position");
        }

        armMotorPower = leftGiven - rightGiven;
        robot.setArmRotatePower(armMotorPower);

        // This section uses the A and B buttons to extend and retract the arm respectively.
        // WARNING: This section currently contains no stoppers, so it is possible to snap the cabling or
        // loosen it an excessive amount. Restraint is to be practiced by driver.
        double spoolMaxPower = 1.0;

        double spoolPower = 0.0;

        if (gamepad1.a){
            spoolPower += spoolMaxPower;
        }
        if(gamepad1.b){
            spoolPower -= spoolMaxPower;
        }

        robot.setSpoolPower(spoolPower);

        // This section uses the triggers to spin the fly wheels for the intake.
        // Right trigger is to intake, and left is to outtake
        double intakePowerMultiplier = 1.0;

        double intakeMax;
        double intakePower = 0.0;

        double leftIntakeGiven = gamepad1.left_trigger;
        double rightIntakeGiven = gamepad1.right_trigger;

        intakeMax = Math.max(leftIntakeGiven,rightIntakeGiven);

        intakePower = rightIntakeGiven - leftIntakeGiven;

        if(intakeMax>1.0) {
            intakePower /= intakeMax;
        }

        intakePower *= intakePowerMultiplier;

        robot.setIntakePower(intakePower);
    }

}
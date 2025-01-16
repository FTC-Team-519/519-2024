package org.firstinspires.ftc.teamcode.util.events;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.util.TritonRobot;

public class DriveSideways implements Event {
    private double distance;
    private TritonRobot robot;

    //drive left is Negative <- mathew said this after he was wrong the first time
    public DriveSideways(double inches, TritonRobot robot) {
        distance = inches*TritonRobot.getCountsPerInchForDriveMotors()/(14.25); //mathew magical constant

        // init robot
        this.robot = robot;
    }

    public void init() {
        this.robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.robot.setDriveTargetPosition((int)distance);
    }

    public void run() {
        if(distance>0) {
            this.robot.getFrontLeft().setPower(0.5);
            this.robot.getFrontRight().setPower(-0.5);
            this.robot.getBackLeft().setPower(-0.5);
            this.robot.getBackRight().setPower(0.5);
        }
        else if(distance<0) {
            this.robot.getFrontLeft().setPower(-0.5);
            this.robot.getFrontRight().setPower(0.5);
            this.robot.getBackLeft().setPower(0.5);
            this.robot.getBackRight().setPower(-0.5);
        }
    }

    public void shutdown() {
        this.robot.setAllDrivePower(0.0d);
        this.robot.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isDone() {
        if(this.robot.getFrontLeft().getTargetPosition()>0) {
            return this.robot.getFrontLeft().getCurrentPosition() >= this.robot.getFrontLeft().getTargetPosition();
        }
        else {
            return this.robot.getFrontLeft().getCurrentPosition() <= this.robot.getFrontLeft().getTargetPosition();
        }
    }
}

package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.util.events.Event;

public class DriveSideways implements Event {
    private double distance;
    private TritonRobot robot;

    DriveSideways(double inches, TritonRobot robot) {
        distance = inches*TritonRobot.getCountsPerInchForDriveMotors();

        // init robot
        this.robot = robot;

        this.robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.robot.setDriveTargetPosition((int)distance);
    }

    public void run() {
        if(isDone()){
            robot.setAllDrivePower(0);
            return;
        }
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

    public boolean isDone() {
        if(this.robot.getFrontLeft().getTargetPosition()>0) {
            return this.robot.getFrontLeft().getCurrentPosition() >= this.robot.getFrontLeft().getTargetPosition();
        }
        else {
            return this.robot.getFrontLeft().getCurrentPosition() <= this.robot.getFrontLeft().getTargetPosition();
        }
    }
}

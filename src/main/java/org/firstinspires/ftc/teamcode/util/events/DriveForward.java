package org.firstinspires.ftc.teamcode.util.events;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.util.TritonRobot;

// Uses encoders
public class DriveForward implements Event {
    private double distance;
    private TritonRobot robot;

    public DriveForward(double inches, TritonRobot robot) {
        distance = inches*TritonRobot.getCountsPerInchForDriveMotors();

        // init robot
        this.robot = robot;
    }

    public void init() {
        this.robot.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.robot.setDriveTargetPosition((int)distance);
    }
//    @Override
    public void run() {
        if(distance>=0) {
            this.robot.setAllDrivePower(1.0);
        } else {
            this.robot.setAllDrivePower(-1.0);
        }
    }

    public void shutdown() {
        this.robot.setAllDrivePower(0.0);
        this.robot.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

//    @Override
    public boolean isDone() {
        if(distance>=0) {
            return robot.atDriveTargetPosition(true);
        } else {
            return robot.atDriveTargetPosition(false);
        }
    }
}

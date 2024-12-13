package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.util.events.Event;

// Uses encoders
public class DriveForward implements Event {
    private double distance;
    private TritonRobot robot;

    DriveForward(double inches, TritonRobot robot) {
        distance = inches*TritonRobot.getCountsPerInchForDriveMotors();

        // init robot
        this.robot = robot;

        this.robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.robot.setDriveTargetPosition((int)distance);
    }
//    @Override
    public void run() {
        this.robot.setAllDrivePower(0.5);
    }

//    @Override
    public boolean isDone() {
        return robot.atDriveTargetPositionForward();
    }
}

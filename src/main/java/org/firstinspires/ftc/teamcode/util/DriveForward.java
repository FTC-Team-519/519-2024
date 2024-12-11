package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.util.events.Event;

// Uses encoders
public class DriveForward implements Event {
    private double distance;
    private TritonRobot robot;

    DriveForward(double inches, TritonRobot robot) {
        distance = inches*TritonRobot.getCountsPerInchForMotors();

        // init robot
        this.robot = robot;

        this.robot.isTargetPositionsForMotorsSet((int)distance);
    }
//    @Override
    public void run() {

    }

//    @Override
    public boolean isDone() {
        return false;
    }
}

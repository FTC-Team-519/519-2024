package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.events.Event;

// Uses encoders
public class DriveForward implements Event {
    private double distance;
    private TritonRobot robot;

    DriveForward(double inches, HardwareMap hmap) {
        distance = inches*TritonRobot.getCountsPerInchForMotors();

        // init robot
        robot = new TritonRobot(hmap);

        robot.setTargetPositionsForMotors(distance);
    }
//    @Override
    public void run() {

    }

//    @Override
    public boolean isDone() {
        return false;
    }
}

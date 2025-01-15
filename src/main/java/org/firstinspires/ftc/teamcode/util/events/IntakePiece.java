package org.firstinspires.ftc.teamcode.util.events;

import org.firstinspires.ftc.teamcode.util.TritonRobot;

public class IntakePiece implements Event{
    private TritonRobot robot;
    final int SAMPLECOUNT = 10;
    private int curr_samples = 0;
    private double speed;

    //speed should be positive
    public IntakePiece(TritonRobot robot, double speed) {
        this.robot = robot;
        this.speed = speed;
    }

    @Override
    public void init() {}

    @Override
    public void run() {
        if(curr_samples<SAMPLECOUNT) {
            robot.setIntakePower(speed);
            if (robot.doesIntakeContainPiece(3.0)) {
                curr_samples++;
            }
        }else{
            robot.setIntakePower(0.0);
        }
    }

    @Override
    public void shutdown() {
        robot.setIntakePower(0.0);
    }

    @Override
    public boolean isDone() {
        this.shutdown();
        return curr_samples>=SAMPLECOUNT;
    }
}

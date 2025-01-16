package org.firstinspires.ftc.teamcode.util.events;

import org.firstinspires.ftc.teamcode.util.TritonRobot;

public class IntakePiece implements Event{
    private TritonRobot robot;
    final int SAMPLECOUNT = 10;
    public int curr_samples = 0;
    private double speed;
    private boolean intake;
    private Timer timer = null;

    //speed should be positive, intake is true, outtake is false
    public IntakePiece(TritonRobot robot, double speed, boolean intake) {
        this.robot = robot;
        this.speed = speed;
        this.intake = intake;
        if (!intake){
            this.timer = new Timer(1000);
        }
    }

    @Override
    public void init() {
        if (this.timer != null) {
            this.timer.init();
            robot.setIntakePower(-speed);
        }else{
            robot.setIntakePower(speed);
        }
    }

    @Override
    public void run() {
        if (intake) {
            robot.setIntakePower(speed);
            if (curr_samples < SAMPLECOUNT) {
                if (robot.doesIntakeContainPiece(3.0)) {
                    curr_samples+=1;
                }
            } else {
                robot.setIntakePower(0.0);
            }
        }else{
            this.timer.run();
            robot.setIntakePower(-speed);
            if (this.timer.isDone()){
                robot.setIntakePower(0.0);
            }
        }
    }

    @Override
    public void shutdown() {
        robot.setIntakePower(0.0);
        if (this.timer != null) {
            this.timer.shutdown();
        }
    }

    @Override
    public boolean isDone() {
        if (!this.intake){
            return timer.isDone();
        }
        return curr_samples>=SAMPLECOUNT;
    }
}

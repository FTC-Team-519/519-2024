package org.firstinspires.ftc.teamcode.util.events;
public class Timer implements Event {
    private double ms;
    private long startTime;
    public Timer(long miliSeconds) {
        ms =  miliSeconds;
    }
    public void init() {
        startTime = System.currentTimeMillis();
    }
    public void run() {}
    public void shutdown() {}
    public boolean isDone() {
        return (System.currentTimeMillis()-startTime)>=ms;
    }
}
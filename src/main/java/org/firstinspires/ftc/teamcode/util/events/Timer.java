package org.firstinspires.ftc.teamcode.util.events;

public class Timer implements Event {
    double ms;
    long startTime;

    Timer(long miliSeconds) {
        ms =  miliSeconds;
        startTime = System.currentTimeMillis();
    }

    public void run() {}
    public boolean isDone() {
        return (System.currentTimeMillis()-startTime)>=ms;
    }
}

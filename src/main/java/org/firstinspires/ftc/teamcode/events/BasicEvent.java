package org.firstinspires.ftc.teamcode.events;

public class BasicEvent implements Event {
    Timer timer;

    public BasicEvent() {
        timer = new Timer(1000);
    }
    public void run() {
        timer.run();
    }
    public boolean isDone() {
        return timer.isDone();
    }
}

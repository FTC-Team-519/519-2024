package org.firstinspires.ftc.teamcode.util.events;

public class BasicEvent implements Event {
    Timer timer;

    public BasicEvent() {
        timer = new Timer(1000);
    }

    public void init() {}

    public void run() {
        timer.run();
    }

    public void shutdown() {}

    public boolean isDone() {
        return timer.isDone();
    }
}

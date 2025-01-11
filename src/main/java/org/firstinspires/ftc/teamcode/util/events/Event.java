package org.firstinspires.ftc.teamcode.util.events;

public interface Event {
    public void init();
    public void run();
    public void shutdown();
    public boolean isDone();
}
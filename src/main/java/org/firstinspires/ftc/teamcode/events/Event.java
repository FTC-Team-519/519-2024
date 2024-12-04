package org.firstinspires.ftc.teamcode.events;

import android.os.Build;

import java.time.Duration;
import java.time.Instant;
import java.util.Timer;

public interface Event {
    public void run();
    public boolean isDone();
}
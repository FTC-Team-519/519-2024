package org.firstinspires.ftc.teamcode.events;

import android.os.Build;

import java.time.Duration;
import java.time.Instant;
import java.util.Timer;

public abstract class Event {
    //timeout<0 means no timeout
    protected int timeout = -1;
    protected Instant start = null;

    public Event(int timeout){
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            this.start = Instant.now();
        }
        this.timeout = 0;
    }

    public abstract void run();
    public boolean isDone(){
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            return (timeout>=0 && (Duration.between(this.start, Instant.now()).toMillis()>=timeout));
        }else{
            return true;
        }
    }
}

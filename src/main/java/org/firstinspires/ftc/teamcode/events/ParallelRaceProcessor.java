package org.firstinspires.ftc.teamcode.events;

import java.util.ArrayList;
import java.util.Arrays;

public class ParallelRaceProcessor implements Event {
    // Runs all simultaneously first done isDone

    ArrayList<Event> events;
    boolean finished = false;

    ParallelRaceProcessor(Event... tasks) {
        // adds tasks to events
        events.addAll(Arrays.asList(tasks));
    }

    // Runs all event in events if event.isDone() events.remove(event)
    public void run() {
        if (isDone()) {
            return;
        }

        for (int i=0; i<events.size(); i++) {
            Event curEvent = events.get(i);
            curEvent.run();

            if(curEvent.isDone()) {
                finished = true;
                return;
            }
        }
    }
    public boolean isDone() {
        return finished;
    }
}

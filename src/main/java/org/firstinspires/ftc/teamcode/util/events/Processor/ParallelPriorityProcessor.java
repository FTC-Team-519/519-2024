package org.firstinspires.ftc.teamcode.util.events;

import java.util.ArrayList;
import java.util.Arrays;

public class ParallelPriorityProcessor implements Event {
    // Runs all simultaneously first done isDone

    ArrayList<Event> events = new ArrayList<>();
    private boolean finished = false;

    public ParallelRaceProcessor(Event... tasks) {
        // adds tasks to events
        events.addAll(Arrays.asList(tasks));
    }

    public void init() {
        for (Event curEvent: events) {
            curEvent.init();
        }
    }

    // Runs all event in events if event.isDone() events.remove(event)
    public void run() {
        if (isDone()) {
            return;
        }

        for (Event curEvent : events) {
            curEvent.run();
        }

        if (events.get(0).isDone()) {
            finished = true;
        }
    }

    public void shutdown() {
        for (Event curEvent: events) {
            curEvent.shutdown();
        }
    }

    public boolean isDone() {
        return finished;
    }
}

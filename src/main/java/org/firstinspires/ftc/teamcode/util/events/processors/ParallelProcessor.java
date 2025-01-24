package org.firstinspires.ftc.teamcode.util.events.processors;

import org.firstinspires.ftc.teamcode.util.events.Event;

import java.util.ArrayList;
import java.util.Arrays;

public class ParallelProcessor implements Event {
    // Runs all simultaneously when all finish, isDone
    // Removes event from events when event.isDone()==true


    ArrayList<Event> events = new ArrayList<>();
    public ParallelProcessor(Event... tasks) {
        // adds tasks to events
        events.addAll(Arrays.asList(tasks));
    }

    public void init() {
        for (Event curEvent : events) {
            curEvent.init();
        }
    }

    // Runs all event in events if event.isDone() events.remove(event)
    // Keep in mind
    public void run() {
        for (int i=0; i<events.size(); i++) {
            Event curEvent = events.get(i);

            if(! curEvent.isDone()) {
                curEvent.run();
            } else {
                curEvent.shutdown();
                events.remove(i);
                i--;
            }
        }
    }

    public void shutdown() {
        for (Event curEvent: events) {
            curEvent.shutdown();
        }
    }

    public boolean isDone() {
        return events.isEmpty();
    }
}

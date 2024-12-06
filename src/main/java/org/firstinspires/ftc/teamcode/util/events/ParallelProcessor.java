package org.firstinspires.ftc.teamcode.util.events;

import java.util.ArrayList;
import java.util.Arrays;

public class ParallelProcessor implements Event {
    // Runs all simultaneously when all finish, isDone
    // Removes event from events when event.isDone()==true

    ArrayList<Event> events;

    ParallelProcessor(Event... tasks) {
        // adds tasks to events
        events.addAll(Arrays.asList(tasks));
    }

    // Runs all event in events if event.isDone() events.remove(event)
    public void run() {
        for (int i=0; i<events.size(); i++) {
            Event curEvent = events.get(i);

            if(! curEvent.isDone()) {
                curEvent.run();
                if(curEvent.isDone()) {
                    events.remove(i);
                    i--;
                }
            }
        }
    }
    public boolean isDone() {
        return events.isEmpty();
    }
}

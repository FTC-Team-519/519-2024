package org.firstinspires.ftc.teamcode.events;

import java.util.ArrayList;
import java.util.Arrays;

public class LinearProcessor implements Event{
    ArrayList<Event> events;
    int currentEvent = 0;

    LinearProcessor(Event... tasks) {
        // adds tasks to events
        events.addAll(Arrays.asList(tasks));
    }

    /* go through all events
    * if event isDone curevent ++
    * if curEvent > events.size() then do nothing
    * */
    public void run() {
        if (events.get(currentEvent).isDone()) {
            currentEvent++;
        }
        if (currentEvent >= events.size()) {
            return;
        }

        events.get(currentEvent).run();
    }

    public boolean isDone() {
        return currentEvent>=events.size();
    }
}

package org.firstinspires.ftc.teamcode.util.events.processors;

import org.firstinspires.ftc.teamcode.util.events.Event;

import java.util.ArrayList;
import java.util.Arrays;

public class LinearProcessor implements Event {
    ArrayList<Event> events = new ArrayList<>();
    int currentEventIndex = 0;
    boolean currentEventIsInitialized = false;


    public LinearProcessor(Event... tasks) {
        // adds tasks to events
        events.addAll(Arrays.asList(tasks));
    }
    /* go through all events
    * if event isDone curevent ++
    * if curEvent > events.size() then do nothing
    * */

    public void init() {}

    public void run() {
        if (isDone()) {
            return;
        }

        Event currentEvent = events.get(currentEventIndex);

        if(!currentEventIsInitialized) {
            currentEvent.init();
            currentEventIsInitialized = true;
        }

        currentEvent.run();

        if (currentEvent.isDone()) {
            currentEvent.shutdown();
            ++currentEventIndex;
            currentEventIsInitialized = false;
            currentEvent.shutdown();
        }
    }

    public void shutdown() {}


    public boolean isDone() {
        return currentEventIndex >= events.size();
    }
}

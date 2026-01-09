package org.ducktapeandaprayer.ftc.lib;

import java.util.concurrent.TimeUnit;

public class TimedEvent {
    private final Runnable function;
    private final long interval;

    private long lastRun = 0;
    public boolean initialized = false;

    public void tick(long currentTime) {
        if(currentTime - lastRun > interval || !initialized) {
            initialized = true;
            function.run();
            lastRun = currentTime;
        }
    }

    /**
     * @param interval the interval that the function is to be called on in ms
     */
    public TimedEvent(long interval, Runnable function) {
        this.function = function;
        this.interval = TimeUnit.NANOSECONDS.convert(interval, TimeUnit.MILLISECONDS);
    }
}

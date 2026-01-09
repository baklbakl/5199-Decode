package org.ducktapeandaprayer.ftc.lib;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.atomic.AtomicLong;

public class LoopTimeCalculator {
    private final ElapsedTime runtime = new ElapsedTime();

    private long lastNanos = 0;
    private final long[] loopTimes = new long[5];
    private int loopTimesPos = 0;
    private final AtomicLong timeNs = new AtomicLong(0);

    public void reset() {
        runtime.reset();

        for(int i = 0; i < loopTimes.length; i++) {
            loopTimes[i] = 0;
        }

        loopTimesPos = 0;
        lastNanos = 0;
    }

    public long getLoopTimeNs() {
        return timeNs.get();
    }

    public double getLoopTime() {
        return getLoopTimeNs() / 1000000000.0d;
    }

    public double getLoopFrequency() {
        return 1/getLoopTime();
    }

    @Override
    public String toString() {
        return "Loop Speed: " + getLoopFrequency() + " Hz\n" +
                "Loop Time: " + getLoopTime() + "s\n";
    }

    public void tick() {
        long thisNanos = runtime.nanoseconds();
        loopTimes[loopTimesPos] = thisNanos - lastNanos;
        loopTimesPos++;
        loopTimesPos %= loopTimes.length;
        long averageNanos = 0;
        for(long time : loopTimes) {
            averageNanos += time;
        }
        averageNanos /= loopTimes.length;
        lastNanos = thisNanos;

        this.timeNs.set(averageNanos);
    }
}

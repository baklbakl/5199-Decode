package org.ducktapeandaprayer.ftc.lib;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicLong;

public class LoopTimeCalculator {
    private final ElapsedTime runtime = new ElapsedTime();

    private long lastNanos = 0;
    private final long[] loopTimes;
    private int loopTimesPos = 0;
    private final AtomicLong timeNs = new AtomicLong(0);
    //private final AtomicLong medianTimeNs = new AtomicLong(0);

    public LoopTimeCalculator(int sampleCount) {
         loopTimes = new long[sampleCount];
    }

    public LoopTimeCalculator() {
        this(5);
    }

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
                //"Median Speed: " + 1000000000.0d / medianTimeNs.get() + " Hz\n";
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
        this.timeNs.set(averageNanos);

        //long[] arrayToSort = Arrays.copyOf(loopTimes, loopTimes.length);
        //Arrays.sort(arrayToSort);
        //medianTimeNs.set(arrayToSort[arrayToSort.length/2]);

        lastNanos = thisNanos;
    }
}

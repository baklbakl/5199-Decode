package net.polarbub.ftc.lib;

import com.qualcomm.robotcore.util.RobotLog;
import net.polarbub.ftc.lib.variablewatcher.VariableWatcher;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public abstract class ThreadEx extends Thread {
    private final AtomicBoolean stop = new AtomicBoolean(false);
    public final LoopTimeCalculator loopTimes = new LoopTimeCalculator();
    protected final List<VariableWatcher> variablesWatched;
    protected final StringBuffer telemetry;
    private final StringBuffer[] telemetryBuffers = new StringBuffer[2];

    public ThreadEx() {
        this.variablesWatched = new ArrayList<>();
        telemetry = new StringBuffer();
    }

    //Only call this from another thread, not this one
    public void stopAndJoin() {
        stop.set(true);

        //We do this instead of this.join() because the app will interrupt the thread calling join()
        while(this.isAlive()) {
            try {
                this.join();
            } catch (InterruptedException e) {
                RobotLog.ii("ThreadEx", "Interrupted while joining");
            }
        }
    }

    @Override
    public void run() {
        loopTimes.reset();

        while(!stop.get()) {
            tick();
            for(VariableWatcher watcher : variablesWatched) {
                watcher.tick();
            }
            loopTimes.tick();

            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    public String getAndClearTelemetry() {
        String out = telemetry.toString();
        telemetry.setLength(0);
        return out;
    }

    public abstract void tick();
}

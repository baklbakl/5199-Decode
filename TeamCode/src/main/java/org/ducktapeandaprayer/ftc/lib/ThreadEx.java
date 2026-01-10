package org.ducktapeandaprayer.ftc.lib;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.VariableWatcher;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public abstract class ThreadEx extends Thread {
    private final AtomicBoolean stop = new AtomicBoolean(false);
    public final LoopTimeCalculator loopTimes = new LoopTimeCalculator();
    protected final List<VariableWatcher> variablesWatched;
    protected List<TimedEvent> timedEvents = new ArrayList<>();
    protected final ElapsedTime runtime = new ElapsedTime();

    private AtomicReference<Double> lastWatchDogFeedTime = new AtomicReference<>(Double.NaN);
    private final double watchdogTimeOut = 1.0;

    private Throwable err = null;
    private final Object errLock = new Object();

    protected StringBuilder telemetry = new StringBuilder();
    private String compiledTelemetry = "";
    private final Object compiledTelemetryLock = new Object();

    public ThreadEx() {
        this.variablesWatched = new ArrayList<>();

        this.setUncaughtExceptionHandler((Thread t, Throwable err) -> {
            synchronized (errLock) {
                this.err = err;
            }
            RobotLog.ii("ThreadEx", "Thread stopping due to exception");
        });
    }

    public void softStop() {
        stop.set(true);
    }

    protected boolean shouldStop() {
        return stop.get();
    }

    //Only call this from another thread, not this one
    public void stopAndJoin() {
        softStop();

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
        runtime.reset();

        while(!stop.get()) {
            if(runtime.seconds() - lastWatchDogFeedTime.get() >= watchdogTimeOut) {
                RobotLog.ii("ThreadEx", "Thread stopping due to watchdog timeout");
                return;
            }

            tick();
            for(VariableWatcher watcher : variablesWatched) {
                watcher.tick();
            }

            long thisNanos = runtime.nanoseconds();
            for(TimedEvent event : timedEvents) {
                event.tick(thisNanos);
            }

            loopTimes.tick();

            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        RobotLog.ii("ThreadEx", "Thread stopping cleanly");
    }

    public Throwable getErr() {
        synchronized (errLock) {
            return err;
        }
    }

    protected void updateTelemetry() {
        synchronized (compiledTelemetryLock) {
            compiledTelemetry = telemetry.toString();
            telemetry = new StringBuilder();

            //Remove trailing newline to make sure that telemetry.addline doesn't have extra newlines in its
            if(!compiledTelemetry.isEmpty() && compiledTelemetry.charAt(compiledTelemetry.length() - 1) == '\n') {
                compiledTelemetry = compiledTelemetry.substring(0, compiledTelemetry.length() - 1);
            }

        }
    }

    public void addTelemetryTo(Telemetry telemetry) {
        synchronized (compiledTelemetryLock) {
            telemetry.addLine(compiledTelemetry);
            compiledTelemetry = "";
        }
    }

    public void feedWatchdog() {
        lastWatchDogFeedTime.set(runtime.seconds());
    }

    public abstract void tick();
}

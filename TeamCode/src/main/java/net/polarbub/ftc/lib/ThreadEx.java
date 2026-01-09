package net.polarbub.ftc.lib;

import com.qualcomm.robotcore.util.RobotLog;
import net.polarbub.ftc.lib.variablewatcher.VariableWatcher;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public abstract class ThreadEx extends Thread {
    private final AtomicBoolean stop = new AtomicBoolean(false);
    public final LoopTimeCalculator loopTimes = new LoopTimeCalculator();
    protected final List<VariableWatcher> variablesWatched;

    protected StringBuilder telemetry = new StringBuilder();
    private String compiledTelemetry = "";
    private final Object compiledTelemetryLock = new Object();

    public ThreadEx() {
        this.variablesWatched = new ArrayList<>();
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

    protected void updateTelemetry() {
        synchronized (compiledTelemetryLock) {
            compiledTelemetry = telemetry.toString();
            telemetry = new StringBuilder();

            //Remove trailing newline to make sure that telemetry.addline doesn't have extra newlines in its
            if(compiledTelemetry.charAt(compiledTelemetry.length() - 1) == '\n') {
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

    public abstract void tick();
}

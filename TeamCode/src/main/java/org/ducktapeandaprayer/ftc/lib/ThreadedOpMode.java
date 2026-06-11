package org.ducktapeandaprayer.ftc.lib;

import com.qualcomm.robotcore.util.RobotLog;
import org.ducktapeandaprayer.ftc.lib.optiondriver.BooleanOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.EnumOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.OptionDriver;

import java.util.HashMap;
import java.util.Map;

public class ThreadedOpMode extends FastIterativeOpMode {
    private final Map<String, ThreadEx> threads = new HashMap<>();

    protected Map<String, ThreadEx> getThreads() {
        return threads;
    }

    private OptionDriver optionDriver;

    public OptionDriver getOptionDriver() {
        return optionDriver;
    }

    private final LoopTimeCalculator timer = new LoopTimeCalculator();

    public Boolean timingDebugInfo = true;
    public volatile Alliance alliance;

    public Alliance getAlliance() {
        return alliance;
    }

    protected void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    protected void init_internal() {
        optionDriver = new OptionDriver();
        try {
            optionDriver.add(new BooleanOption("Loop Timing debug info", "triangle", gamepad1, this.getClass().getField("timingDebugInfo"), this));
            optionDriver.add(new EnumOption<>("Alliance", "cross", gamepad1, this.getClass().getField("alliance"), this, Alliance.class));
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }

        threads.clear();
    }

    @Override
    protected void init_loop_internal() {
        String out = optionDriver.tick();
        if(out != null) {
            telemetry.addLine(out);
            telemetry.update();

            StringBuilder sb = new StringBuilder();
            for(ThreadEx thread : threads.values()) {
                String message = thread.optionDriverUpdateCallback();
                if(message != null) {
                    sb.append(message);
                }
            }

            String threadTelemCompiled = sb.toString();
            if(!threadTelemCompiled.isEmpty()) {
                telemetry.addLine(out);
                telemetry.addLine(threadTelemCompiled);
                telemetry.update();
            }
        }

        try {
            Thread.sleep(1);
        } catch (InterruptedException e) {}
    }

    @Override
    protected void start_internal() {
        for(ThreadEx t : threads.values()) {
            t.start();
        }

        timer.reset();
    }

    @Override
    protected void loop_internal() {
        try {
            for(Map.Entry<String, ThreadEx> element : threads.entrySet()) {
                ThreadEx thread = element.getValue();
                Throwable err = thread.getErr();
                if(err != null) {
                    //Clean up after ourselves
                    stop_internal();
                    RobotLog.ii("ThreadedTele", "Stopping due to error");
                    throw new RuntimeException("Error in " + element.getKey(), err);
                }

                thread.addTelemetryTo(telemetry);
                thread.feedWatchdog();
            }

            telemetry.addLine();
            if(timingDebugInfo) {
                for(Map.Entry<String, ThreadEx> t : threads.entrySet()) {
                    telemetry.addData("Timings for " + t.getKey(), t.getValue().loopTimes);
                }

                telemetry.addData("Main Loop Timings", timer.toString());
            }

            telemetry.update();

            timer.tick();

            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        } catch (Exception e) {
            stop_internal();
            if(e instanceof RuntimeException && e.getCause() instanceof InterruptedException && !opModeIsActive()) {
                RobotLog.ee("ThreadedTele", "Ignored exception as its an InterruptedException probably from stopping the opmode");
                RobotLog.logStackTrace(e);
            } else {
                throw e;
            }
        }
    }

    @Override
    protected void stop_internal() {
        for(ThreadEx t : threads.values()) {
            t.softStop();
        }

        for(ThreadEx t : threads.values()) {
            t.stopAndJoin();
        }
    }
}

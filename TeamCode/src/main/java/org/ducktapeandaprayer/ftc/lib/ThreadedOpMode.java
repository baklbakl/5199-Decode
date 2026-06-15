package org.ducktapeandaprayer.ftc.lib;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
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

    public Vector2d invertIfBlue(Vector2d vector) {
        if(getAlliance() == Alliance.BLUE) {
            return new Vector2d(vector.x, -vector.y);
        }
        return vector;
    }

    public Pose2d invertIfBlue(Pose2d pose) {
        if(getAlliance() == Alliance.BLUE) {
            return new Pose2d(invertIfBlue(pose.position), pose.heading.inverse());
        }
        return pose;
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

    public String optionDriverUpdateCallback() {
        return null;
    }
    public String additionalInitLoopTelemetry() {
        return null;
    }

    private String lastOptionDriverTelemetry = null;

    @Override
    protected void init_loop_internal() {
        String out = optionDriver.tick();
        if(out != null) {
            lastOptionDriverTelemetry = out;
            telemetry.addLine(lastOptionDriverTelemetry);
            telemetry.update();

            StringBuilder threadTelemetry = new StringBuilder();
            String message = this.optionDriverUpdateCallback();
            if(message != null) {
                threadTelemetry.append(message);
            }

            for(ThreadEx thread : threads.values()) {
                String otherThreadMessage = thread.optionDriverUpdateCallback();
                if(otherThreadMessage != null) {
                    threadTelemetry.append(otherThreadMessage);
                }
            }

            String threadTelemCompiled = threadTelemetry.toString();
            if(!threadTelemCompiled.isEmpty()) {
                StringBuilder sb = new StringBuilder();
                sb.append(out).append('\n');
                sb.append(threadTelemCompiled).append('\n');
                lastOptionDriverTelemetry = sb.toString();
                telemetry.addLine(lastOptionDriverTelemetry);
                telemetry.update();
            }
        }

        String additionalTelem = additionalInitLoopTelemetry();
        if (additionalTelem != null) {
            if(lastOptionDriverTelemetry != null) {
                telemetry.addLine(lastOptionDriverTelemetry);
            }
            telemetry.addLine(additionalTelem);
            telemetry.update();
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

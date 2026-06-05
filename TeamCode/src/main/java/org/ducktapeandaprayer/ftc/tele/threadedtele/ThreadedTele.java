package org.ducktapeandaprayer.ftc.tele.threadedtele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import org.ducktapeandaprayer.ftc.MainBotConfig;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.Alliance;
import org.ducktapeandaprayer.ftc.lib.FastIterativeOpMode;
import org.ducktapeandaprayer.ftc.lib.LoopTimeCalculator;
import org.ducktapeandaprayer.ftc.lib.ThreadEx;
import org.ducktapeandaprayer.ftc.lib.optiondriver.BooleanOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.EnumOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.OptionDriver;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "1ThreadedTele", group = MainBotConfig.opmodeGroupName)
public class ThreadedTele extends FastIterativeOpMode {
    public MainBotHardware hardware = null;
    private final Map<String, ThreadEx> threads = new HashMap<>();
    private final LoopTimeCalculator timer = new LoopTimeCalculator();
    private OptionDriver optionDriver;

    //Never change these after the threads start
    public volatile Alliance alliance = Alliance.lastAlliance;
    public Boolean timingDebugInfo = true;




    @Override
    protected void init_internal() {
        hardware = MainBotHardware.getOrCreate(hardwareMap);
        optionDriver = new OptionDriver();
        try {
            optionDriver.add(new BooleanOption("Loop Timing debug info", "triangle", gamepad1, this.getClass().getField("timingDebugInfo"), this));
            optionDriver.add(new EnumOption<>("Alliance", "cross", gamepad1, this.getClass().getField("alliance"), this, Alliance.class));
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }

        threads.clear();
        ControlHubThread controlHubThread = new ControlHubThread(hardware, this, gamepad1, optionDriver);
        ExpansionHubThread expansionHubThread = new ExpansionHubThread(hardware, this, controlHubThread, gamepad1, optionDriver);

        threads.put("control hub thread", controlHubThread);
        threads.put("expansion hub thread", expansionHubThread);
    }

    @Override
    protected void init_loop_internal() {
        String out = optionDriver.tick();
        if(out != null) {
            telemetry.addLine(out);
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

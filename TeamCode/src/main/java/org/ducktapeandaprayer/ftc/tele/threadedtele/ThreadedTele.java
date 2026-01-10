package org.ducktapeandaprayer.ftc.tele.threadedtele;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.RobotLog;
import org.ducktapeandaprayer.ftc.MainBotConfig;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.Alliance;
import org.ducktapeandaprayer.ftc.lib.FastIterativeOpMode;
import org.ducktapeandaprayer.ftc.lib.LoopTimeCalculator;
import org.ducktapeandaprayer.ftc.lib.ThreadEx;
import org.ducktapeandaprayer.ftc.lib.optiondriver.OptionDriver;

import java.util.HashMap;
import java.util.Map;
import java.util.stream.Collectors;

@TeleOp(group = MainBotConfig.opmodeGroupName)
public class ThreadedTele extends FastIterativeOpMode {
    public MainBotHardware hardware = null;
    private ControlHubThread controlHubThread = null;
    private ExpansionHubThread exansionHubThread = null;
    private Map<String, ThreadEx> threads = new HashMap<>();
    private LoopTimeCalculator timer = new LoopTimeCalculator();
    private OptionDriver optionDriver;

    @Override
    protected void init_internal() {
        hardware = MainBotHardware.getOrCreate(hardwareMap);
        optionDriver = new OptionDriver();

        controlHubThread = new ControlHubThread(hardware, gamepad1);
        exansionHubThread = new ExpansionHubThread(hardware, gamepad1, optionDriver);

        threads.put("control hub thread", controlHubThread);
        threads.put("expansion hub thread", exansionHubThread);
    }

    @Override
    protected void init_loop_internal() {
        String out = optionDriver.tick();
        if(out != null) {
            telemetry.addLine(out);
            telemetry.update();

            if (exansionHubThread.alliance == Alliance.BLUE) {
                hardware.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            } else if (exansionHubThread.alliance == Alliance.RED) {
                hardware.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
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
        for(Map.Entry<String, ThreadEx> t : threads.entrySet()) {
            Throwable err = t.getValue().getErr();
            if(err != null) {
                //Clean up after ourselves
                stop_internal();
                RobotLog.ii("ThreadedTele", "Stopping due to error");
                throw new RuntimeException("Error in " + t.getKey(), err);
            }

            t.getValue().addTelemetryTo(telemetry);
        }

        for(Map.Entry<String, ThreadEx> t : threads.entrySet()) {
            telemetry.addData("Timings for " + t.getKey(), t.getValue().loopTimes);

            t.getValue().feedWatchdog();
        }

        telemetry.addData("Main Loop Timings", timer.toString());
        telemetry.update();
        timer.tick();

        if(gamepad1.dpadUpWasPressed()) {
            RobotLog.ii("ThreadeTele", hardwareMap.voltageSensor.entrySet().stream().map(Map.Entry::getKey).collect(Collectors.toList()).toString());
        }

        try {
            Thread.sleep(1);
        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
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

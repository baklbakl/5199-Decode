package org.ducktapeandaprayer.ftc.tele.threadedtele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.ducktapeandaprayer.ftc.MainBotConfig;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.FastIterativeOpMode;
import org.ducktapeandaprayer.ftc.lib.LoopTimeCalculator;

@TeleOp(group = MainBotConfig.opmodeGroupName)
public class ThreadedTele extends FastIterativeOpMode {
    public MainBotHardware hardware = null;
    private ControlHubThread controlHubThread = null;
    private LoopTimeCalculator timer = new LoopTimeCalculator();

    @Override
    protected void init_internal() {
        hardware = MainBotHardware.getOrCreate(hardwareMap);
    }

    @Override
    protected void init_loop_internal() {
        controlHubThread = new ControlHubThread(hardware, gamepad1);

    }

    @Override
    protected void start_internal() {
        controlHubThread.start();
        timer.reset();
    }

    @Override
    protected void loop_internal() {
        controlHubThread.addTelemetryTo(telemetry);
        telemetry.addData("Main Loop Timings", timer.toString());
        telemetry.addData("Control Hub Loop Timings", controlHubThread.loopTimes);

        timer.tick();

        try {
            Thread.sleep(1);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    protected void stop_internal() {
        controlHubThread.stopAndJoin();
    }
}

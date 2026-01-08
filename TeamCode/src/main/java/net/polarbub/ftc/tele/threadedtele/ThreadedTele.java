package net.polarbub.ftc.tele.threadedtele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import net.polarbub.ftc.MainBotConfig;
import net.polarbub.ftc.hardware.MainBotHardware;
import net.polarbub.ftc.lib.FastIterativeOpMode;

@TeleOp(group = MainBotConfig.opmodeGroupName)
public class ThreadedTele extends FastIterativeOpMode {
    public MainBotHardware hardware = null;
    private ControlHubThread controlHubThread = null;

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
    }

    @Override
    protected void loop_internal() {
        telemetry.addData("Control Hub Loop Timings", controlHubThread.loopTimes);
        telemetry.addLine(controlHubThread.getAndClearTelemetry());
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

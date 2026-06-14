package org.ducktapeandaprayer.ftc.lib.action;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.Supplier;

public class WaitUntilAction extends SetupAction {
    private final Supplier<Boolean> done;
    private final Double timeout;
    private final ElapsedTime elapsedTime;

    public WaitUntilAction(Supplier<Boolean> done, Double timeout) {
        this.done = done;
        this.timeout = timeout;
        elapsedTime = new ElapsedTime();
    }

    @Override
    public void setup(@NonNull TelemetryPacket telemetryPacket) {
        elapsedTime.reset();
    }

    @Override
    public boolean subRun(@NonNull TelemetryPacket telemetryPacket) {
        if(done.get()) {
            return false;
        }

        return timeout == null || !(elapsedTime.seconds() > timeout);
    }
}

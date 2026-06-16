package org.ducktapeandaprayer.ftc.lib.action;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.Supplier;

public class TimeoutAction extends SetupAction {
    private final Action toRun;
    private final Double timeout;
    private final ElapsedTime elapsedTime;

    public TimeoutAction(Action toRun, Double timeout) {
        this.toRun = toRun;
        this.timeout = timeout;
        elapsedTime = new ElapsedTime();
    }

    @Override
    public void setup(@NonNull TelemetryPacket telemetryPacket) {
        elapsedTime.reset();
    }

    @Override
    public boolean subRun(@NonNull TelemetryPacket telemetryPacket) {
        if(!toRun.run(telemetryPacket)) {
            return false;
        }

        return timeout == null || !(elapsedTime.seconds() > timeout);
    }
}

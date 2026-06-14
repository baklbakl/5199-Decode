package org.ducktapeandaprayer.ftc.lib;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public abstract class SetupAction implements Action {
    public abstract void setup(@NonNull TelemetryPacket telemetryPacket);
    public abstract boolean subRun(@NonNull TelemetryPacket telemetryPacket);

    private boolean setup = false;

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if(!setup) {
            setup(telemetryPacket);
            setup = true;
        }

        return subRun(telemetryPacket);
    }
}

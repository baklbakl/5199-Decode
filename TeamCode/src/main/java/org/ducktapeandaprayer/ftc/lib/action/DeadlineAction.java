package org.ducktapeandaprayer.ftc.lib.action;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class DeadlineAction extends SetupAction {
    @Override
    public void setup(@NonNull TelemetryPacket telemetryPacket) {

    }

    @Override
    public boolean subRun(@NonNull TelemetryPacket telemetryPacket) {
        return false;
    }
}
